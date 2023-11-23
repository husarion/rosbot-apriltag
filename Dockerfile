# Use an official ROS 2 base image
FROM husarnet/ros:humble-ros-base

# Install pip and other dependencies
RUN apt-get update && apt-get install -y \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        ros-$ROS_DISTRO-tf-transformations && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install transforms3d

# Create a workspace
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# Copy your Python script to the workspace
COPY follow_apriltag /ros2_ws/src/

# Install dependencies
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd /ros2_ws; colcon build'

# Source the workspace
SHELL ["/bin/bash", "-c"]
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Run the node
CMD ["ros2", "run", "follow_apriltag", "follow_apriltag.py"]
