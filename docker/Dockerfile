FROM achilleas2942/ros:noetic

ENV PATH="/root/.cargo/bin:${PATH}"

# Required dependencies
RUN apt-get update && apt-get install -y \
    python3-gi \
    ros-noetic-cv-bridge \
    tmux \
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

# Install RUST and Gstreamer dependencies
RUN curl --proto '=https' --tlsv1.2 https://sh.rustup.rs -sSf | bash -s -- -y

# Create an entrypoint script to pull the latest updates and build the workspace
COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

# Source ROS and workspace setup files
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set the entrypoint
ENTRYPOINT ["/root/entrypoint.sh"]

CMD ["$(hostname -I | awk '{print $1}')", "$(hostname -I | awk '{print $1}')"]
