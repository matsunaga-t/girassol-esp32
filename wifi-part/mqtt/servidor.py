from flask import Flask, request, jsonify, render_template
import sqlite3
from datetime import datetime

app = Flask(__name__)
DB  = "leituras.db"

# ── cria tabela com todos os campos do PID ──────────────────────
def init_db():
    conn = sqlite3.connect(DB)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS leituras (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            ldr1_raw    REAL,
            ldr2_raw    REAL,
            ldr1_mean   REAL,
            ldr2_mean   REAL,
            pid_error   REAL,
            pid_output  INTEGER,
            pwm_value   INTEGER,
            timestamp   TEXT
        )
    """)
    conn.commit()
    conn.close()
    print("[DB] banco inicializado em", DB)

# ── serve o dashboard no browser ───────────────────────────────
@app.route("/")
def index():
    return render_template("dashboard.html")

# ── recebe JSON do ESP32 ────────────────────────────────────────
@app.route("/dados", methods=["POST"])
def receber_dados():
    dados = request.get_json(force=True)
    agora = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    conn = sqlite3.connect(DB)
    conn.execute("""
        INSERT INTO leituras
          (ldr1_raw, ldr2_raw, ldr1_mean, ldr2_mean,
           pid_error, pid_output, pwm_value, timestamp)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
    """, (
        dados.get("ldr1_raw",   0),
        dados.get("ldr2_raw",   0),
        dados.get("ldr1_mean",  0),
        dados.get("ldr2_mean",  0),
        dados.get("pid_error",  0),
        dados.get("pid_output", 0),
        dados.get("pwm_value",  0),
        agora
    ))
    conn.commit()
    conn.close()

    print(f"[{agora}] ldr1={dados.get('ldr1_raw',0):.3f}"
          f" ldr2={dados.get('ldr2_raw',0):.3f}"
          f" err={dados.get('pid_error',0):.3f}"
          f" out={dados.get('pid_output',0)}")

    return jsonify({"status": "ok"}), 200

# ── retorna histórico para o dashboard ─────────────────────────
@app.route("/historico", methods=["GET"])
def historico():
    conn = sqlite3.connect(DB)
    cursor = conn.execute("""
        SELECT ldr1_raw, ldr2_raw, ldr1_mean, ldr2_mean,
               pid_error, pid_output, pwm_value, timestamp
        FROM leituras
        ORDER BY id DESC
        LIMIT 100
    """)
    rows = cursor.fetchall()
    conn.close()

    resultado = [{
        "ldr1_raw":   r[0],
        "ldr2_raw":   r[1],
        "ldr1_mean":  r[2],
        "ldr2_mean":  r[3],
        "pid_error":  r[4],
        "pid_output": r[5],
        "pwm_value":  r[6],
        "timestamp":  r[7]
    } for r in rows]

    return jsonify(resultado)

# ── inicia ──────────────────────────────────────────────────────
if __name__ == "__main__":
    init_db()
    app.run(
        host="0.0.0.0",   # aceita conexões de toda a rede AP
        port=5000,
        debug=True
    )
