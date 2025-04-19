#!/bin/bash

SCRIPT_PATH=$(realpath $0)
SCRIPT_DIR=$(dirname $SCRIPT_PATH)
source $SCRIPT_DIR/env.sh

STATUS_FILE="/images/status.json"
CONFIG_FILE="/config/clients.json"

LAST_STATUS=""
APP_CONTAINERS=()

get_apps_for_sender_ip() {
    jq -r --arg ip "$SENDER_IP" '.[$ip][]?' "$CONFIG_FILE"
}

start_apps() {
    DIRECTORY="$1"
    for APP in $(get_apps_for_sender_ip); do
        echo "Starting container: $APP for sender $SENDER_IP"
        docker run -d --rm \
            --name "${APP}_${SENDER_IP//./_}" \
            -v "$DIRECTORY:/data" \
            -e DATA_PATH="/data" \
            my-image-repo/$APP
        APP_CONTAINERS+=("${APP}_${SENDER_IP//./_}")
    done
}

stop_apps() {
    for CONTAINER in "${APP_CONTAINERS[@]}"; do
        echo "Stopping container: $CONTAINER"
        docker stop "$CONTAINER"
    done
    APP_CONTAINERS=()
}

while true; do
    if [[ -f "$STATUS_FILE" ]]; then
        STATUS=$(jq -r .status "$STATUS_FILE")
        DIRECTORY=$(jq -r .directory "$STATUS_FILE")

        if [[ "$STATUS" != "$LAST_STATUS" ]]; then
            echo "Status changed: $STATUS for $SENDER_IP"

            if [[ "$STATUS" == "started" ]]; then
                start_apps "$DIRECTORY"
            elif [[ "$STATUS" == "stopped" ]]; then
                stop_apps
            fi

            LAST_STATUS=$STATUS
        fi
    fi
    sleep 2
done
