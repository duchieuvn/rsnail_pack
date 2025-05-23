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
    libtbb-dev \
    python3-ament-package \
    python3-colcon-common-extensions \
    libqt5serialport5-dev \
    libqt5serialbus5 \
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
# gtsam 4.2.0-ros
# COPY ./gtsam /root/gtsam

WORKDIR /root
RUN git clone --branch 5.15 https://github.com/qt/qtserialbus.git
RUN git clone https://github.com/1r0b1n0/libros2qt.git
RUN git clone --branch 4.2.0-ros https://github.com/borglab/gtsam.git

WORKDIR /root/qtserialbus
RUN mkdir -p build && cd build && qmake .. && make && make install

WORKDIR /root/libros2qt
RUN mkdir -p build && cd build && \
    source /opt/ros/humble/setup.bash && \
    cmake .. && \
    make && \
    make install

WORKDIR /root/gtsam
RUN mkdir -p build && cd build && \ 
    cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_USE_TBB=OFF && \
    make -j2 && \
    make install

WORKDIR /root/as_pipeline-test_can_sender

# Launch shell when container starts
CMD ["/bin/bash"]
