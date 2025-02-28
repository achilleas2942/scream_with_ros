export ECN_ENABLED=0

if (($ECN_ENABLED == 1)); then
    export SET_ECN="set-ecn=1"
    export SCREAMTX_PARAM_ECT="-ect 1"
    export RETRIEVE_ECN="retrieve-ecn=true"
    
    MY_GST_INSTALL=$(readlink -f $SCRIPT_DIR/../../../udp_gstreamer/install)
    echo "MY_GST_INSTALL=$MY_GST_INSTALL"
    
    export GST_PLUGIN_PATH=$MY_GST_INSTALL/lib/x86_64-linux-gnu
    export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$MY_GST_INSTALL/lib/x86_64-linux-gnu/gstreamer-1.0
    export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gstreamer-1.0
    
    export LD_LIBRARY_PATH=$MY_GST_INSTALL/lib/x86_64-linux-gnu
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MY_GST_INSTALL/lib/x86_64-linux-gnu/gstreamer-1.0
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu/gstreamer-1.0
    
    export PATH=$MY_GST_INSTALL/bin:$PATH
    export PKG_CONFIG_PATH=$MY_GST_INSTALL/lib/x86_64-linux-gnu/pkgconfig
    export PYTHONPATH=${PYTHONPATH}:$MY_GST_INSTALL/lib/python3/site-packages/
    
    LP=$(pkg-config --libs-only-L gstreamer-1.0 )
    export RUSTFLAGS="$LP $RUSTFLAGS"
fi

SCREAMLIB_DIR=$SCRIPT_DIR/../../code/wrapper_lib
SCREAM_TARGET_DIR=$SCRIPT_DIR/../target/debug/
export GST_PLUGIN_PATH=$SCREAM_TARGET_DIR:$GST_PLUGIN_PATH
export LD_LIBRARY_PATH=$SCREAMLIB_DIR:$LD_LIBRARY_PATH

echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
echo "GST_PLUGIN_PATH=$GST_PLUGIN_PATH"

SENDER_IP=127.0.0.1
RECEIVER_IP=127.0.0.1

PORT0_RTP=30112
PORT0_RTCP=30113

USE_SCREAM=1
