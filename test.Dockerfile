FROM ros:humble-ros-base-jammy

# Install dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    x11-apps \
    ros-humble-ament-cmake \
    ros-humble-rclcpp \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-pcl-conversions \
    ros-humble-rviz2 \
    ros-humble-can-msgs \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-generator-c \
    libpcl-dev \
    libeigen3-dev \
    libcgal-dev \
    python3-ament-package \
    python3-colcon-common-extensions \
    libqt5serialbus5-dev \
    qt5-qmake \
    qtbase5-dev \
    qtchooser \
    qtbase5-private-dev \
    && rm -rf /var/lib/apt/lists/*

# Set up environment
ENV DISPLAY=:0
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set default shell
SHELL ["/bin/bash", "-c"]

COPY ./as_pipeline-test_can_sender /root/as_pipeline-test_can_sender
# qtserial 5.15
COPY ./qtserialbus /root/qtserialbus
# gtsam 4.2.0-ros
COPY ./gtsam /root/gtsam
COPY ./libros2qt /root/libros2qt

# Build libros2qt
WORKDIR /root/libros2qt
RUN mkdir -p build && cd build && cmake .. && make

# Set working directory
WORKDIR /root/as_pipeline-test_can_sender

# Launch shell when container starts
CMD ["/bin/bash"]
