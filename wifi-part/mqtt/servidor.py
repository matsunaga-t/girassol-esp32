"""
servidor.py — Flask + Mosquitto (paho-mqtt)
Recebe dados do ESP32 via MQTT, salva no SQLite,
serve o dashboard e permite ajuste de ganhos pelo browser.

Instalar: pip3 install flask paho-mqtt
"""
import json
import sqlite3
import threading
import queue
from datetime import datetime

import paho.mqtt.client as mqtt
from flask import Flask, jsonify, render_template, request, Response, stream_with_context

# ── configurações ──────────────────────────────────────────────
MQTT_BROKER   = "localhost"   # broker Mosquitto no próprio Pi
MQTT_PORT     = 1883
TOPIC_DADOS   = "pid/dados"   # ESP32 publica aqui
TOPIC_CONFIG  = "pid/config"  # Flask publica aqui → ESP32 recebe
DB            = "leituras.db"

app = Flask(__name__)

# fila compartilhada entre thread MQTT e SSE
sse_queue: queue.Queue = queue.Queue(maxsize=50)

# ── banco de dados ─────────────────────────────────────────────
def init_db():
    conn = sqlite3.connect(DB)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS leituras (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            ldr1_raw    REAL, ldr2_raw    REAL,
            ldr1_mean   REAL, ldr2_mean   REAL,
            pid_error   REAL,
            pid_output  INTEGER,
            pwm_value   INTEGER,
            kp          REAL, ki REAL, kd REAL,
            timestamp   TEXT
        )
    """)
    conn.commit()
    conn.close()
    print(f"[DB] banco pronto: {DB}")

def salvar_leitura(d: dict):
    agora = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    conn = sqlite3.connect(DB)
    conn.execute("""
        INSERT INTO leituras
          (ldr1_raw, ldr2_raw, ldr1_mean, ldr2_mean,
           pid_error, pid_output, pwm_value, kp, ki, kd, timestamp)
        VALUES (?,?,?,?,?,?,?,?,?,?,?)
    """, (
        d.get("ldr1_raw",   0), d.get("ldr2_raw",   0),
        d.get("ldr1_mean",  0), d.get("ldr2_mean",  0),
        d.get("pid_error",  0), d.get("pid_output", 0),
        d.get("pwm_value",  0),
        d.get("kp", 0), d.get("ki", 0), d.get("kd", 0),
        agora
    ))
    conn.commit()
    conn.close()
    d["timestamp"] = agora
    return d

# ── callbacks MQTT ─────────────────────────────────────────────
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[MQTT] conectado ao broker ({MQTT_BROKER}:{MQTT_PORT})")
        client.subscribe(TOPIC_DADOS)
        print(f"[MQTT] subscrito em '{TOPIC_DADOS}'")
    else:
        print(f"[MQTT] falha na conexão rc={rc}")

def on_message(client, userdata, msg):
    try:
        dados = json.loads(msg.payload.decode())
        registro = salvar_leitura(dados)
        print(f"[{registro['timestamp']}] "
              f"ldr1={dados.get('ldr1_raw',0):.3f} "
              f"ldr2={dados.get('ldr2_raw',0):.3f} "
              f"err={dados.get('pid_error',0):.3f} "
              f"out={dados.get('pid_output',0)}")
        # envia para todos os clientes SSE conectados
        try:
            sse_queue.put_nowait(registro)
        except queue.Full:
            pass
    except Exception as e:
        print(f"[MQTT] erro ao processar mensagem: {e}")

# ── thread MQTT ────────────────────────────────────────────────
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

def iniciar_mqtt():
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    mqtt_client.loop_forever()

# ── rotas Flask ────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template("dashboard.html")

@app.route("/historico")
def historico():
    conn = sqlite3.connect(DB)
    cursor = conn.execute("""
        SELECT ldr1_raw, ldr2_raw, ldr1_mean, ldr2_mean,
               pid_error, pid_output, pwm_value, kp, ki, kd, timestamp
        FROM leituras ORDER BY id DESC LIMIT 100
    """)
    rows = cursor.fetchall()
    conn.close()
    cols = ["ldr1_raw","ldr2_raw","ldr1_mean","ldr2_mean",
            "pid_error","pid_output","pwm_value","kp","ki","kd","timestamp"]
    return jsonify([dict(zip(cols, r)) for r in rows])

@app.route("/stream")
def stream():
    """Server-Sent Events — push em tempo real para o browser."""
    def gerar():
        # cada cliente tem sua própria fila de eventos
        q = queue.Queue(maxsize=20)
        # registra na lista global de clientes SSE
        sse_clients.append(q)
        try:
            while True:
                try:
                    dado = q.get(timeout=30)
                    yield f"data: {json.dumps(dado)}\n\n"
                except queue.Empty:
                    yield ": heartbeat\n\n"  # mantém conexão viva
        finally:
            sse_clients.remove(q)
    return Response(
        stream_with_context(gerar()),
        mimetype="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"}
    )

@app.route("/config", methods=["POST"])
def config():
    """Recebe novos ganhos do dashboard e publica em pid/config."""
    dados = request.get_json(force=True)
    kp = dados.get("kp")
    ki = dados.get("ki")
    kd = dados.get("kd")

    partes = []
    if kp is not None: partes.append(f"kp={kp}")
    if ki is not None: partes.append(f"ki={ki}")
    if kd is not None: partes.append(f"kd={kd}")

    if not partes:
        return jsonify({"status": "erro", "msg": "nenhum ganho informado"}), 400

    mensagem = ",".join(partes)
    mqtt_client.publish(TOPIC_CONFIG, mensagem)
    print(f"[config] publicado em '{TOPIC_CONFIG}': {mensagem}")
    return jsonify({"status": "ok", "mensagem": mensagem})

# lista global de filas SSE (uma por cliente conectado)
sse_clients: list = []

# dispatcher: copia da sse_queue global para cada cliente
def dispatcher():
    while True:
        dado = sse_queue.get()
        for q in list(sse_clients):
            try:
                q.put_nowait(dado)
            except queue.Full:
                pass

# ── main ───────────────────────────────────────────────────────
if __name__ == "__main__":
    init_db()

    # thread MQTT (daemon — encerra com o processo principal)
    t_mqtt = threading.Thread(target=iniciar_mqtt, daemon=True)
    t_mqtt.start()

    # thread dispatcher SSE
    t_disp = threading.Thread(target=dispatcher, daemon=True)
    t_disp.start()

    print("[Flask] iniciando em http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
