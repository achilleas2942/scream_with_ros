#!/bin/bash
SCRIPT_PATH=$(realpath $0)
SCRIPT_DIR=$(dirname $SCRIPT_PATH)
source $SCRIPT_DIR/env.sh

QUEUE="queue max-size-buffers=2 max-size-bytes=0 max-size-time=0"

if (($USE_SCREAM == 1)); then
    SCREAMRX0="! screamrx name=screamrx0 screamrx0.src "
    SCREAMRX0_RTCP="screamrx0.rtcp_src ! f0."
else
    SCREAMRX0=""
    SCREAMRX0_RTCP=""
fi

# Pipe the received LiDAR data into a named pipe (FIFO) for the Python ROS publisher
FIFO_PATH="/tmp/lidar_fifo"
rm -f $FIFO_PATH
mkfifo $FIFO_PATH

LIDAR_PROCESS="filesink location=$FIFO_PATH sync=false"

export RECVPIPELINE="rtpbin latency=10 name=r 
udpsrc port=$PORT0_RTP address=$RECEIVER_IP $RETRIEVE_ECN ! 
 queue $SCREAMRX0 ! application/x-raw,media=lidar ! r.recv_rtp_sink_0 r. ! $QUEUE ! $LIDAR_PROCESS 
 r.send_rtcp_src_0 ! funnel name=f0 ! queue ! udpsink host=$SENDER_IP port=$PORT0_RTCP sync=false async=false 
 $SCREAMRX0_RTCP udpsrc port=$PORT0_RTCP ! r.recv_rtcp_sink_0 "

export GST_DEBUG="screamrx:2"
killall -9 scream_receiver
$SCREAM_TARGET_DIR/scream_receiver &

# Start the Python script to read from FIFO and publish to ROS
tmux new-session -d -s lidar_ros_publisher "python3 $SCRIPT_DIR/lidar_publisher.py $FIFO_PATH"
