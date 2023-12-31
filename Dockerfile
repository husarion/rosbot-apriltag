# Use an official ROS 2 base image
FROM husarnet/ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

# Install pip and other dependencies
RUN apt-get update && apt-get install -y \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        ros-$ROS_DISTRO-tf-transformations \
        ros-${ROS_DISTRO}-apriltag-ros # for messages definitions && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install transforms3d

# Create a workspace
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# Copy your Python script to the workspace
COPY follow_apriltag /ros2_ws/src/follow_apriltag

# Install dependencies
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install

# Environment variable for AprilTag frame
ENV APRIL_TAG_FRAME="tag36h11:0"

# Run the node
CMD ["ros2", "run", "follow_apriltag", "follow_apriltag"]
