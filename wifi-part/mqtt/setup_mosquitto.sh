#!/bin/bash
# ================================================================
#  setup_mosquitto.sh — Instala e configura o Mosquitto no Pi
#  Execute com: bash setup_mosquitto.sh
# ================================================================

echo "=== [1/4] Instalando Mosquitto ==="
sudo apt update
sudo apt install -y mosquitto mosquitto-clients

echo ""
echo "=== [2/4] Configurando o broker ==="
sudo tee /etc/mosquitto/conf.d/pid.conf > /dev/null << 'EOF'
# Aceita conexões sem autenticação na rede local (porta 1883)
listener 1883
allow_anonymous true

# Log de conexões (opcional — comente para reduzir verbosidade)
log_type connect
log_type disconnect
EOF

echo ""
echo "=== [3/4] Habilitando e iniciando o serviço ==="
sudo systemctl enable mosquitto
sudo systemctl restart mosquitto
sudo systemctl status mosquitto --no-pager

echo ""
echo "=== [4/4] Instalando dependência Python ==="
pip3 install flask paho-mqtt --break-system-packages

echo ""
echo "=== PRONTO ==="
echo "Broker Mosquitto rodando em 192.168.4.1:1883"
echo ""
echo "Para testar:"
echo "  Subscriber: mosquitto_sub -h localhost -t 'pid/#' -v"
echo "  Publisher:  mosquitto_pub -h localhost -t 'pid/config' -m 'kp=0.5,ki=0.01,kd=0'"
