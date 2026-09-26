FROM ros:lyrical-ros-base@sha256:a2dbd538c006b7187b60d6afcaa8eace1f0864e8c2649a654013f5057841355d

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pip \
      python3-venv \
      ros-lyrical-robot-state-publisher \
      ros-lyrical-rosbag2 \
      ros-lyrical-rviz2 \
      ros-lyrical-cv-bridge \
      ros-lyrical-message-filters \
      ros-lyrical-nav-msgs \
      ros-lyrical-tf2-ros \
      ros-lyrical-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/frc_venv
ENV PATH="/opt/frc_venv/bin:${PATH}"

COPY jetson/requirements.txt /tmp/frc-requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/frc-requirements.txt

WORKDIR /workspaces/frc_autonomy
COPY jetson/ros2_ws/ ./
RUN source /opt/ros/lyrical/setup.bash \
    && colcon build --symlink-install

COPY jetson/docker/entrypoint.sh /frc-entrypoint.sh
RUN chmod +x /frc-entrypoint.sh

ENTRYPOINT ["/frc-entrypoint.sh"]
CMD ["ros2", "launch", "frc_bringup", "foundation.launch.py"]
