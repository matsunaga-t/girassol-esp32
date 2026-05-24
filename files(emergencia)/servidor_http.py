"""
servidor_http.py — Versão hotspot do celular
Flask puro, sem MQTT. O Pi conecta no hotspot e recebe
IP dinâmico do roteador do celular.

Instalar: pip3 install flask --break-system-packages
Rodar:    python3 servidor_http.py

Ao iniciar, o IP atual do Pi é exibido no terminal.
Anote esse IP e use no ESP32 e no browser do notebook.
"""
import json
import sqlite3
import socket
from datetime import datetime
from flask import Flask, jsonify, render_template, request

DB  = "leituras.db"
app = Flask(__name__)

# ── utilitário: descobre o IP local do Pi na rede ──────────────
def get_local_ip():
    try:
        # abre socket UDP para o Google DNS — não envia nada,
        # mas o SO preenche o IP local correto na interface ativa
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "IP não encontrado — verifique a conexão Wi-Fi"

# ── banco ───────────────────────────────────────────────────────
def init_db():
    conn = sqlite3.connect(DB)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS leituras (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            solar_v     REAL,
            solar_a     REAL,
            ldr1_raw    REAL,  ldr2_raw    REAL,
            ldr1_mean   REAL,  ldr2_mean   REAL,
            pid_error   REAL,
            pid_output  INTEGER,
            pwm_value   INTEGER,
            timestamp   TEXT
        )
    """)
    conn.commit()
    conn.close()
    print("[DB] banco pronto:", DB)

# ── rotas ───────────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template("dashboard.html")

@app.route("/ip")
def ip_atual():
    """Rota extra: retorna o IP atual do Pi em JSON.
    O dashboard usa isso para se auto-configurar."""
    return jsonify({"ip": get_local_ip()})

@app.route("/dados", methods=["POST"])
def receber_dados():
    d     = request.get_json(force=True)
    agora = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    conn  = sqlite3.connect(DB)
    conn.execute("""
        INSERT INTO leituras
          (solar_v, solar_a,
           ldr1_raw, ldr2_raw, ldr1_mean, ldr2_mean,
           pid_error, pid_output, pwm_value, timestamp)
        VALUES (?,?,?,?,?,?,?,?,?,?)
    """, (
        d.get("solar_v",   0), d.get("solar_a",   0),
        d.get("ldr1_raw",  0), d.get("ldr2_raw",  0),
        d.get("ldr1_mean", 0), d.get("ldr2_mean", 0),
        d.get("pid_error", 0),
        d.get("pid_output",0), d.get("pwm_value", 0),
        agora
    ))
    conn.commit()
    conn.close()
    print(f"[{agora}] V={d.get('solar_v',0):.2f}V A={d.get('solar_a',0):.3f}A "
          f"ldr1={d.get('ldr1_raw',0):.3f} ldr2={d.get('ldr2_raw',0):.3f} "
          f"err={d.get('pid_error',0):.3f}")
    return jsonify({"status": "ok"}), 200

@app.route("/historico")
def historico():
    conn   = sqlite3.connect(DB)
    cursor = conn.execute("""
        SELECT solar_v, solar_a,
               ldr1_raw, ldr2_raw, ldr1_mean, ldr2_mean,
               pid_error, pid_output, pwm_value, timestamp
        FROM leituras ORDER BY id DESC LIMIT 300
    """)
    rows = cursor.fetchall()
    conn.close()
    cols = ["solar_v","solar_a","ldr1_raw","ldr2_raw","ldr1_mean","ldr2_mean",
            "pid_error","pid_output","pwm_value","timestamp"]
    return jsonify([dict(zip(cols, r)) for r in rows])

# ── main ────────────────────────────────────────────────────────
if __name__ == "__main__":
    init_db()
    ip = get_local_ip()
    print("")
    print("╔══════════════════════════════════════════════╗")
    print("║   Solar PID Dashboard — Hotspot do celular   ║")
    print("╠══════════════════════════════════════════════╣")
    print(f"║   IP do Raspberry Pi : {ip:<22}║")
    print(f"║   Dashboard          : http://{ip}:5000  ║")
    print(f"║   ESP32 SERVER_URL   : http://{ip}:5000/dados  ║")
    print("╚══════════════════════════════════════════════╝")
    print("")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
