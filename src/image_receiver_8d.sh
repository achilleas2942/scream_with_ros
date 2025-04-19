#!/bin/bash
SCRIPT_PATH=$(realpath $0)
SCRIPT_DIR=$(dirname $SCRIPT_PATH)
source $SCRIPT_DIR/env.sh

# Config
STATUS_FILE="/images/status.json"
WATCHDOG_TIMEOUT=20  # Seconds of no new files = stop
MONITOR_DIR="/images"

# Init status file
echo "{\"status\": \"started\", \"timestamp\": \"$(date +%s)\", \"directory\": \"$MONITOR_DIR\"}" > "$STATUS_FILE"

# Run the pipeline in background
export RECVPIPELINE="rtpbin latency=10 name=r \
udpsrc port=$PORT0_RTP address=$RECEIVER_IP $RETRIEVE_ECN ! \
    queue ! screamrx name=screamrx0 screamrx0.src ! application/x-rtp, media=video, encoding-name=H${ENC_ID}, clock-rate=90000 ! r.recv_rtp_sink_0 \
    r. ! rtph${ENC_ID}depay ! h${ENC_ID}parse ! avdec_h264 name=videodecoder0 ! videoconvert ! jpegenc ! multifilesink location=$MONITOR_DIR/image_%05d.jpg \
    r.send_rtcp_src_0 ! funnel name=f0 ! queue ! udpsink host=$SENDER_IP port=$PORT0_RTCP sync=false async=false \
    screamrx0.rtcp_src ! f0. \
    udpsrc port=$PORT0_RTCP ! r.recv_rtcp_sink_0"

export GST_DEBUG="screamrx:2"

killall -9 scream_receiver 2>/dev/null
$SCREAM_TARGET_DIR/scream_receiver &
RECEIVER_PID=$!

# Watchdog loop to detect inactivity
LAST_COUNT=$(ls $MONITOR_DIR/*.jpg 2>/dev/null | wc -l)
TIMER=0

while kill -0 $RECEIVER_PID 2>/dev/null; do
    sleep 2
    NEW_COUNT=$(ls $MONITOR_DIR/*.jpg 2>/dev/null | wc -l)
    if [[ "$NEW_COUNT" -gt "$LAST_COUNT" ]]; then
        TIMER=0
        LAST_COUNT=$NEW_COUNT
    else
        TIMER=$((TIMER + 2))
    fi

    if [[ "$TIMER" -ge "$WATCHDOG_TIMEOUT" ]]; then
        echo "⚠️ Timeout reached. Stopping receiver..."
        kill $RECEIVER_PID
        break
    fi
done

# Write stopped status
echo "{\"status\": \"stopped\", \"timestamp\": \"$(date +%s)\", \"directory\": \"$MONITOR_DIR\"}" > "$STATUS_FILE"
