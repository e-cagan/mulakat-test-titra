############################
# ROS 2 Humble + PX4 SITL  #
############################
FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]

# --- temel paketler ---
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      git wget zip python3-pip ninja-build \
      protobuf-compiler libeigen3-dev libopencv-dev libprotobuf-dev \
      gazebo python3-pytest && \
    rm -rf /var/lib/apt/lists/*

# --- eski launch klasörlerini sil ---
RUN rm -rf /opt/ros/humble/lib/python3.10/site-packages/launch* \
           /opt/ros/humble/lib/python3.10/site-packages/launch_testing*

# --- tam launch setini pip’ten kur ---
RUN python3 -m pip install --no-cache-dir \
      typing_extensions osrf-pycommon \
      git+https://github.com/ros2/launch.git#subdirectory=launch \
      git+https://github.com/ros2/launch.git#subdirectory=launch_testing \
      git+https://github.com/ros2/launch.git#subdirectory=launch_pytest

# --- pytest plug-in alias ---
RUN python3 - <<'PY'
import site, pathlib
pkg = pathlib.Path(site.getsitepackages()[0]) / "launch_testing"
pkg.mkdir(exist_ok=True)
(pkg / "pytest_plugin.py").write_text("from launch_pytest import *\n")
PY

# --- wrapper: source ROS + öne PYTHONPATH ---
RUN printf '%s\n' \
    '#!/usr/bin/env bash' \
    'source /opt/ros/humble/setup.bash' \
    'export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:$PYTHONPATH' \
    'exec "$@"' \
    > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# --- çalışma alanı & PX4 ---
WORKDIR /ros2_ws
RUN mkdir -p src
RUN git clone --depth 1 --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git
WORKDIR /ros2_ws/PX4-Autopilot
RUN pip3 install --no-cache-dir pyros-genmsg empy jinja2
RUN sed -i 's/math::max(PTHREAD_STACK_MIN, PX4_STACK_ADJUSTED(wq->stacksize))/math::max((size_t)PTHREAD_STACK_MIN, (size_t)PX4_STACK_ADJUSTED(wq->stacksize))/g' \
        platforms/common/px4_work_queue/WorkQueueManager.cpp
RUN make px4_sitl

# --- proje ve köprü paketleri ---
WORKDIR /ros2_ws/src
COPY . .
RUN git clone https://github.com/PX4/px4_ros_com.git
RUN git clone https://github.com/PX4/px4_msgs.git

CMD ["bash"]
