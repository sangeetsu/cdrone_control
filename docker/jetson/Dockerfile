ARG IMAGE_NAME=dev:mavros2
FROM ${IMAGE_NAME}

# install deps
RUN apt-get update && apt-get install -y \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

RUN pip install \
    numpy \
    scipy \
    norfair \
    ultralytics \
    pyyaml \
    setuptools==58.2.0

# Copy packages
COPY ros2/src/drone_msgs /root/ros2_ws/src/drone_msgs
COPY ros2/src/drone_bringup /root/ros2_ws/src/drone_bringup
COPY ros2/src/drone_behavior_pkg /root/ros2_ws/src/drone_behavior_pkg
COPY ros2/src/drone_control_pkg /root/ros2_ws/src/drone_control_pkg
COPY ros2/src/drone_light_pkg /root/ros2_ws/src/drone_light_pkg
COPY ros2/src/drone_vision_pkg /root/ros2_ws/src/drone_vision_pkg

WORKDIR /root/ros2_ws

# build packages
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --packages-select \
      drone_msgs \
      drone_bringup \
      drone_behavior_pkg \
      drone_control_pkg \
      drone_light_pkg \
      drone_vision_pkg"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /root/ros2_ws/install/local_setup.bash" >> ~/.bashrc

CMD ["bash"]
