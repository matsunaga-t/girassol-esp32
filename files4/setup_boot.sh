#!/bin/bash
# ================================================================
#  setup_boot.sh — IP fixo + autostart do dashboard no boot
#  Execute UMA VEZ com: bash setup_boot.sh
#  Depois disso o Pi sobe tudo automaticamente ao ligar.
# ================================================================

set -e  # para se qualquer comando falhar

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   Setup: IP fixo + autostart dashboard           ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""

# ── 1. IP fixo na interface wlan0 ──────────────────────────────
echo "[1/3] Configurando IP fixo 192.168.4.1 em wlan0..."

# Remove entradas antigas para wlan0 se existirem
sudo sed -i '/^interface wlan0/,/^$/d' /etc/dhcpcd.conf

# Adiciona configuração de IP fixo no final
sudo tee -a /etc/dhcpcd.conf > /dev/null << 'EOF'

interface wlan0
    static ip_address=192.168.4.1/24
    nohook wpa_supplicant
EOF

echo "    → /etc/dhcpcd.conf atualizado"

# ── 2. Serviço systemd para o dashboard ───────────────────────
echo "[2/3] Criando serviço systemd dashboard.service..."

# Detecta usuário atual para usar no serviço
CURRENT_USER=$(whoami)
DASHBOARD_PATH="/home/${CURRENT_USER}/dashboard"

sudo tee /etc/systemd/system/dashboard.service > /dev/null << EOF
[Unit]
Description=Solar PID Dashboard
After=network.target mosquitto.service
Wants=mosquitto.service

[Service]
User=${CURRENT_USER}
WorkingDirectory=${DASHBOARD_PATH}
ExecStart=/usr/bin/python3 ${DASHBOARD_PATH}/servidor.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

echo "    → /etc/systemd/system/dashboard.service criado"
echo "    → usuário: ${CURRENT_USER}"
echo "    → pasta:   ${DASHBOARD_PATH}"

# ── 3. Habilita e inicia ───────────────────────────────────────
echo "[3/3] Habilitando serviços no boot..."

sudo systemctl daemon-reload
sudo systemctl enable mosquitto
sudo systemctl enable dashboard.service

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   Pronto! Reinicie para aplicar tudo:            ║"
echo "║                                                  ║"
echo "║   sudo reboot                                    ║"
echo "║                                                  ║"
echo "║   Após reiniciar:                                ║"
echo "║   • Rede pi-dashboard disponível                 ║"
echo "║   • Dashboard em http://192.168.4.1:5000         ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""
echo "Comandos úteis:"
echo "  Ver logs ao vivo:  sudo journalctl -u dashboard.service -f"
echo "  Status:            sudo systemctl status dashboard.service"
echo "  Reiniciar serviço: sudo systemctl restart dashboard.service"
