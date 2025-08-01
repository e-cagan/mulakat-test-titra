# ROS 2 Humble temel imajı
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

# temel araçlar
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    git wget zip python3-pip ninja-build protobuf-compiler \
    libeigen3-dev libopencv-dev libprotobuf-dev gazebo \
    python3-pytest \
  && rm -rf /var/lib/apt/lists/*

# PX4-Autopilot
WORKDIR /root
RUN git clone --depth 1 --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git
WORKDIR /root/PX4-Autopilot
RUN pip3 install --no-cache-dir pyros-genmsg empy jinja2
RUN sed -i 's/math::max(PTHREAD_STACK_MIN, PX4_STACK_ADJUSTED(wq->stacksize))/math::max((size_t)PTHREAD_STACK_MIN, (size_t)PX4_STACK_ADJUSTED(wq->stacksize))/g' \
    platforms/common/px4_work_queue/WorkQueueManager.cpp
RUN make px4_sitl

# ROS 2 çalışma alanını kopyala
WORKDIR /root/ros2_ws
COPY src/ src/

# PX4-ROS paketleri
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/PX4/px4_ros_com.git
RUN git clone https://github.com/PX4/px4_msgs.git

# **rosdep** adımını atlıyoruz (zaten container çalışırken colcon build yapacağız)
# Böylece test_package’ın pytest’i çözümlemeye çalışması engellenmiş oldu.
RUN apt-get update && \
    apt-get install -y ros-humble-launch-testing ros-humble-launch-testing-ros

CMD ["/bin/bash"]
