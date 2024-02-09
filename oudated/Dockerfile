FROM arm64v8/ros:humble

# # install ros package
# RUN apt-get update && apt-get install -y \
#       ros-${ROS_DISTRO}-demo-nodes-cpp \
#       ros-${ROS_DISTRO}-demo-nodes-py && \
#     rm -rf /var/lib/apt/lists/*

# # launch ros package
# CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]

# WORKDIR /usr/src/app
#SHELL bash /ros_entrypoint.sh
# SHELL ["/bin/bash", "-c"]

RUN mkdir -p robot/src

WORKDIR /usr/src/app/robot/src
RUN mkdir volum

# COPY src/interfaces interfaces
# COPY src/visual_landing_py visual_landing_py
COPY ros_entrypoint . 

#SHELL ["/bin/bash", "-c", "/ros_entrypoint.sh"]
RUN /ros_entrypoint colcon build
RUN export ROS_LOCAL_HOST=0
ENTRYPOINT ["./ros_entrypoint"]
