from flask import Flask, request, jsonify, render_template
import sqlite3
from datetime import datetime

app = Flask(__name__)
DB = "pid_leituras.db"

# ── Banco de dados ────────────────────────────────────────────
def init_db():
    conn = sqlite3.connect(DB)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS leituras (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp   TEXT,
            ldr1_raw    REAL,
            ldr2_raw    REAL,
            ldr1_mean   REAL,
            ldr2_mean   REAL,
            erro        REAL,
            pid_output  INTEGER,
            pwm_pct     REAL
        )
    """)
    conn.commit()
    conn.close()

# ── Rota: dashboard ───────────────────────────────────────────
@app.route("/")
def index():
    return render_template("dashboard.html")

# ── Rota: recebe dados do ESP32 ───────────────────────────────
@app.route("/dados", methods=["POST"])
def receber_dados():
    d = request.get_json()
    if not d:
        return jsonify({"erro": "JSON inválido"}), 400

    agora = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    conn = sqlite3.connect(DB)
    conn.execute("""
        INSERT INTO leituras
            (timestamp, ldr1_raw, ldr2_raw, ldr1_mean, ldr2_mean, erro, pid_output, pwm_pct)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
    """, (
        agora,
        d.get("ldr1_raw",   0),
        d.get("ldr2_raw",   0),
        d.get("ldr1_mean",  0),
        d.get("ldr2_mean",  0),
        d.get("erro",       0),
        d.get("pid_output", 0),
        d.get("pwm_pct",    0),
    ))
    conn.commit()
    conn.close()

    print(f"[{agora}] erro={d.get('erro',0):.3f}  PID={d.get('pid_output',0)}  PWM={d.get('pwm_pct',0):.1f}%")
    return jsonify({"status": "ok"}), 200

# ── Rota: histórico para o dashboard ─────────────────────────
@app.route("/historico")
def historico():
    n = request.args.get("n", 60, type=int)
    conn = sqlite3.connect(DB)
    cur = conn.execute("""
        SELECT timestamp, ldr1_raw, ldr2_raw, ldr1_mean, ldr2_mean,
               erro, pid_output, pwm_pct
        FROM leituras
        ORDER BY id DESC
        LIMIT ?
    """, (n,))
    rows = cur.fetchall()
    conn.close()

    keys = ["timestamp", "ldr1_raw", "ldr2_raw", "ldr1_mean",
            "ldr2_mean", "erro", "pid_output", "pwm_pct"]
    resultado = [dict(zip(keys, r)) for r in rows]
    return jsonify(resultado)

# ── Rota: última leitura (para cards ao vivo) ─────────────────
@app.route("/ultimo")
def ultimo():
    conn = sqlite3.connect(DB)
    cur = conn.execute("""
        SELECT timestamp, ldr1_raw, ldr2_raw, ldr1_mean, ldr2_mean,
               erro, pid_output, pwm_pct
        FROM leituras ORDER BY id DESC LIMIT 1
    """)
    row = cur.fetchone()
    conn.close()
    if not row:
        return jsonify({})
    keys = ["timestamp", "ldr1_raw", "ldr2_raw", "ldr1_mean",
            "ldr2_mean", "erro", "pid_output", "pwm_pct"]
    return jsonify(dict(zip(keys, row)))

# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    init_db()
    print("Servidor iniciado em http://192.168.4.1:5000")
    app.run(host="0.0.0.0", port=5000, debug=True)
