# ROS 2 Humble'ın temel imajını kullan
FROM osrf/ros:humble-desktop

# Gerekli kurulumları yap, apt'yi temizle
SHELL ["/bin/bash", "-c"]
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    git \
    wget \
    zip \
    python3-pip \
    ninja-build \
    protobuf-compiler \
    libeigen3-dev \
    libopencv-dev \
    libprotobuf-dev \
    protobuf-compiler \
    gazebo \
    && rm -rf /var/lib/apt/lists/*

# PX4-Autopilot'u v1.12.3 olarak kur
WORKDIR /root
RUN git clone --depth 1 --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git
WORKDIR /root/PX4-Autopilot

# Gerekli Python paketlerini kur
RUN pip3 install --no-cache-dir pyros-genmsg empy jinja2

# Hata veren C++ kaynak kodunu derlemeden önce otomatik olarak düzelt.
RUN sed -i 's/math::max(PTHREAD_STACK_MIN, PX4_STACK_ADJUSTED(wq->stacksize))/math::max((size_t)PTHREAD_STACK_MIN, (size_t)PX4_STACK_ADJUSTED(wq->stacksize))/g' platforms/common/px4_work_queue/WorkQueueManager.cpp

# Derlemeyi başlat
RUN make px4_sitl

# ROS 2 çalışma alanını kopyala
WORKDIR /root/ros2_ws
COPY assessment_ws/ /root/ros2_ws/src/

# PX4-ROS köprü paketlerini kaynak kodundan indir.
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/PX4/px4_ros_com.git
RUN git clone https://github.com/PX4/px4_msgs.git

# ROS 2 ortamını source et ve bağımlılıkları kur
WORKDIR /root/ros2_ws
RUN source /opt/ros/humble/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src -y --ignore-src

# Çalışma alanını derle (artık px4_ros_com da derlenecek)
RUN source /opt/ros/humble/setup.bash && colcon build

# Container başladığında bash'i çalıştır ve ROS ortamını hazırla
CMD ["/bin/bash"]
