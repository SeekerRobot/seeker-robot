# Start with the ROS 2 Jazzy base image
FROM ros:jazzy-ros-base AS base

# Add build arguments for user selection (use existing ubuntu user)
ARG USER_NAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000

# 1. Install dependencies (Root)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    sudo \
    openssh-client \
    curl \
    unzip \
    ca-certificates \
    wget \
    coreutils \
    python3-pip \
    gdb \
    iputils-ping \
    net-tools \
    usbutils \
    build-essential \
    nano \
    yq \
    git-lfs \
    pipx \
    # Qt/X11 runtime dependencies
    libxcb1 \
    libx11-xcb1 \
    libxcb-glx0 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    # Install git lfs
    && git lfs install \
    # Download and install cloudflared
    && CLOUDFLARED_ARCH=$(dpkg --print-architecture | sed 's/amd64/amd64/g; s/arm64/arm64/g') \
    && wget -q https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-${CLOUDFLARED_ARCH}.deb -O cloudflared.deb \
    && apt-get install -y ./cloudflared.deb \
    && rm cloudflared.deb \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 2. Use existing user/group
# The base image already provides the `ubuntu` user (UID 1000). No need to create users here.

# Setup directory
RUN mkdir -p /home/$USER_NAME/ros2_workspaces/src

# 3. Clone micro-ROS setup (Done as root to avoid permission jumping, chown later)
WORKDIR /home/$USER_NAME/ros2_workspaces
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Install dependencies for the workspace
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

# Fix permissions so the user/group own their workspace
RUN chown -R $USER_UID:$USER_GID /home/$USER_NAME || true

# Configure passwordless sudo for the user
RUN echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" | tee /etc/sudoers.d/$USER_NAME && \
    chmod 0440 /etc/sudoers.d/$USER_NAME

# --- Switch to existing ubuntu user ---
USER $USER_NAME

RUN rosdep update

# 4. Build the Setup Workspace
RUN /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /home/$USER_NAME/ros2_workspaces && \
    colcon build && \
    rm -rf build log"

# 5. Create and Build the Agent
# Note: This creates a nested workspace 'microros_agent_ws' inside 'ros2_workspaces'
RUN /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash && \
    . /home/$USER_NAME/ros2_workspaces/install/setup.bash && \
    cd /home/$USER_NAME/ros2_workspaces && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    rm -rf microros_agent_ws/build microros_agent_ws/log"

# 6. Update .bashrc
RUN echo "" >> /home/$USER_NAME/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER_NAME/.bashrc && \
    echo "source /home/$USER_NAME/ros2_workspaces/install/setup.bash" >> /home/$USER_NAME/.bashrc

# 7. Install PlatformIO via pipx
# Note: We manually set ENV PATH so 'pio' is immediately available in this container's
# runtime without requiring a shell restart or sourcing .bashrc
ENV PATH="/home/$USER_NAME/.local/bin:$PATH"
RUN pipx install platformio && \
    pipx inject platformio pyyaml && \
    pipx ensurepath

USER root
# 8. Install any other python3 libraries here
# Ensure gpiozero is available for Raspberry Pi GPIO control
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-gpiozero \
        python3-lgpio \
        python3-rpi.gpio \
        python3-serial \
        python3-requests && \
    rm -rf /var/lib/apt/lists/*

FROM base AS dev

# Switch to root to install Gazebo Harmonic and required tools, then switch back
USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-gz-ros2-control \
    curl \
    lsb-release \
    gnupg \
    ca-certificates && \
    # Fetch and install OSRF package signing key
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    # Add Gazebo (OSRF) apt repository for the current distro
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    # Install Gazebo harmonic and needed Qt/X11/XCB/OpenGL libs to support GUI
    apt-get install -y --no-install-recommends \
    gz-harmonic && \
    # Cleanup apt caches
    apt-get clean && rm -rf /var/lib/apt/lists/*

    # Install rqt graph
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-rqt-graph && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

USER $USER_NAME
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

FROM base AS prod

USER $USER_NAME
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]