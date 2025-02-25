FROM achilleas2942/ros:noetic

ENV PATH="/root/.cargo/bin:${PATH}"

# ros required dependencies
RUN apt-get update && apt-get install -y \
    python3-gi \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    --no-install-recommends && rm -rf /var/lib/apt/lists/* 

COPY ./ ./simple_scream_ros_integration
COPY entrypoint.sh /root/entrypoint.sh

# COPY ./deps/rds_poc_msgs/ ./rds_poc_msgs/
WORKDIR /home/ros/simple_scream_ros_integration/gstscream/ros2_ws

# install RUST and Gstreamer dependencies
RUN apt-get update \ 
    && curl --proto '=https' --tlsv1.2 https://sh.rustup.rs -sSf | bash -s -- -y



WORKDIR /home/ros/simple_scream_ros_integration/gstscream/



RUN ./scripts/build.sh

WORKDIR /home/ros/simple_scream_ros_integration/gstscream/ros2_ws

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build 

RUN echo "export ROS_DOMAIN_ID=2" >> ~/.bashrc
RUN echo ". /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo ". /home/ros/simple_scream_ros_integration/gstscream/ros2_ws/install/local_setup.bash" >> ~/.bashrc

ENTRYPOINT [ "/entrypoint.sh" ]

STOPSIGNAL SIGKILL
# CMD "echo $(pwd) && ls -l"

