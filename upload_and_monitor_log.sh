#!/bin/bash

# --- KONFIGURATION ---
# --- STANDARDS ---
DEFAULT_FILTER="all"
DEFAULT_SEARCH="watchdog"
DEFAULT_BAUD="115200"
DEVICE="/dev/ttyUSB0"
DEFAULT_CHIP="esp32 --no-stub"

# Definieren der Variablen
PROJECT_NAME="Renogy-Battery-ESP32-Bluetooth-MQTT-Bridge"
# --- ARGUMENTE ZUWEISEN ---
# $1 = Grep all oder filter, $2 = Suchbegriff, $3 = Baudrate, $4 = Prj Name
MODE=${1:-$DEFAULT_FILTER}
SEARCH_TERM=${2:-$DEFAULT_SEARCH}
BAUD=${3:-$DEFAULT_BAUD}
PROJECT_NAME=${4:-$PROJECT_NAME}
CHIP=${5:-$DEFAULT_CHIP}

BIN_PATH=$(echo ~/Arduino/$PROJECT_NAME*.ino.bin)
echo "=========================================="
echo "📡  ESP32 UPLOAD AND LOG MONITOR"
echo "=========================================="
echo "📍 Project:   $PROJECT_NAME"
echo "📍 Bin path   $BIN_PATH"
echo "📍 Port:   $DEVICE"
echo "🚀 Speed:  $BAUD Baud"
echo "🔍 Filter: '$SEARCH_TERM'"
echo "🔍 CHIP: '$CHIP'"
echo "------------------------------------------"
echo "Abbruch mit STRG+C"

# 1. Prüfen, ob der Sender bereits läuft, beim Upload muss der SOCAT immer geschlossen werden
# Wir suchen nach einem socat Prozess, der dieses DEVICE nutzt
EXISTING_PID=$(pgrep -f "socat.*$DEVICE")
# -n prüft, ob die Variable NICHT leer ist (also eine PID gefunden wurde)
if [ -n "$EXISTING_PID" ]; then
    echo "🛑 Schließe socat-Sender (PID: $EXISTING_PID), damit der Upload starten kann."
    kill "$EXISTING_PID" 2>/dev/null

    # Optional: Kurz warten, bis der Port wirklich frei ist
    sleep 1
else
    echo "✅ Kein laufender socat-Prozess gefunden. Port ist frei."
fi

esptool --chip $CHIP \
  --port /dev/ttyUSB0 \
  --baud 921600 \
  write_flash -z 0x10000 \
  "$BIN_PATH"

sleep 1
# 1. Prüfen, ob der Sender bereits läuft
# Wir suchen nach einem socat Prozess, der dieses DEVICE nutzt
EXISTING_PID=$(pgrep -f "socat.*$DEVICE")

if [ -z "$EXISTING_PID" ]; then
    echo "🚀 Starte neuen Sender für $DEVICE..."
    # Wir nutzen nohup, damit socat stabil läuft, falls das Terminal schließt
    nohup socat -u "$DEVICE",b"$BAUD",raw,echo=0 UDP4-DATAGRAM:127.255.255.255:9999,broadcast,reuseaddr > /dev/null 2>&1 &
    SOCAT_PID=$!
    # Wir markieren, dass DIESE Instanz den Prozess gestartet hat
    I_STARTED_IT=true
else
    echo "✅ Sender läuft bereits (PID: $EXISTING_PID). Nutze bestehende Verbindung."
    SOCAT_PID=$EXISTING_PID
    I_STARTED_IT=false
fi

# Funktion zum sauberen Beenden
cleanup() {
    echo -e "\nBeende Monitoring..."

    if [ "$I_STARTED_IT" = true ]; then
        echo "🛑 Schließe socat-Sender (PID: $SOCAT_PID), da ich ihn gestartet habe."
        kill "$SOCAT_PID" 2>/dev/null
    else
        echo "ℹ️  Lasse socat-Sender laufen, da er von einer anderen Instanz genutzt wird."
    fi

    exit
}
trap cleanup SIGINT SIGTERM

# 2. Empfänger mit IF-Logik starten
if [ "$MODE" == "filter" ]; then
    # Zeigt NUR Zeilen an, die den Begriff enthalten
#    socat UDP4-RECV:9999,reuseaddr - | grep -a --color=always --line-buffered "$SEARCH_TERM"
    socat UDP4-RECV:9999,reuseaddr - | grep -a --color=always --line-buffered "$SEARCH_TERM" | tee -a "${SEARCH_TERM}.log"
else
    # Zeigt ALLES an, markiert den Begriff aber farbig
    socat UDP4-RECV:9999,reuseaddr - | grep -a --color=always --line-buffered -E "$SEARCH_TERM|$" | tee -a "monitor_logs.log"
fi
