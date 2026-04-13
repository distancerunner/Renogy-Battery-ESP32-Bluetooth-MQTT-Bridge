#!/bin/bash
# Inhalt von deploy.env
#SOURCE_PATH=""
#REMOTE_USER=""
#REMOTE_HOST=""
#REMOTE_PATH=""
#FLASH_SCRIPT=""

# Pfad zur .env Datei (gleiches Verzeichnis wie das Skript)
ENV_FILE="$(dirname "$0")/.env"
# Prüfen, ob die .env Datei existiert
if [ -f "$ENV_FILE" ]; then
    source "$ENV_FILE"
else
    echo "❌ Fehler: $ENV_FILE nicht gefunden!"
    exit 1
fi

echo "--- 1. Starte Upload zu $REMOTE_HOST ---"
echo "--- Sourcepath $SOURCE_PATH ---"
echo "--- Remotepath $REMOTE_PATH/$REMOTE_PROJECTNAME/ ---"

# rsync Befehl mit deinen Parametern
rsync -avz --delete --progress \
    --include='*/' \
   --include='*.ino' \
   --include='*.cpp' \
   --include='*.h' \
   --include='*.bin' \
   --exclude='build/**/intermediates/**' \
    --exclude='.git/' \
    "$SOURCE_PATH" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH/$REMOTE_PROJECTNAME/"

# Prüfen, ob der rsync-Befehl erfolgreich beendet wurde (Exit-Code 0)
if [ $? -eq 0 ]; then
    echo -e "\n✅ Upload erfolgreich!"
    echo "--- 2. Führe Flash-Skript auf Remote-Client aus ---"
    
    # Per SSH einloggen und das Skript auf dem asus13-Rechner starten
    ssh "$REMOTE_USER@$REMOTE_HOST" "bash $FLASH_SCRIPT all watchdog 115200 $REMOTE_PROJECTNAME"
else
    echo -e "\n❌ Fehler beim rsync-Upload! Flash-Vorgang abgebrochen."
    exit 1
fi
