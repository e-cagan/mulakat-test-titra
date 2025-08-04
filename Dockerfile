# --- Baz ROS Humble + araçlar ---
FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
      git wget zip python3-pip ninja-build protobuf-compiler \
      libeigen3-dev libopencv-dev libprotobuf-dev gazebo \
      python3-pytest \
   && rm -rf /var/lib/apt/lists/*

# --- Çalışma alanı ---
WORKDIR /ros2_ws
RUN mkdir -p src

# PX4 (aynen kalsın)
RUN git clone --depth 1 --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git
WORKDIR /ros2_ws/PX4-Autopilot
RUN pip3 install --no-cache-dir pyros-genmsg empy jinja2
RUN sed -i 's/math::max(PTHREAD_STACK_MIN, PX4_STACK_ADJUSTED(wq->stacksize))/math::max((size_t)PTHREAD_STACK_MIN, (size_t)PX4_STACK_ADJUSTED(wq->stacksize))/g' \
    platforms/common/px4_work_queue/WorkQueueManager.cpp
RUN make px4_sitl

# --- Proje kodu ---
WORKDIR /ros2_ws/src
COPY . .

# PX4-ROS ek paketleri
RUN git clone https://github.com/PX4/px4_ros_com.git
RUN git clone https://github.com/PX4/px4_msgs.git

# Test bağımlılıkları
RUN apt-get update && apt-get install -y ros-humble-launch-testing ros-humble-launch-testing-ros && \
    rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
