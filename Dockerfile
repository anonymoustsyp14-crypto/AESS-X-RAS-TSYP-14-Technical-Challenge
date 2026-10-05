FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV TURTLEBOT3_MODEL=waffle_pi

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-turtlebot3-description \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-xacro \
    python3-yaml \
    python3-colcon-common-extensions \
    python3-rosdep \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

WORKDIR /workspace/tsyp_ws
COPY tsyp_simulation_phase1/src /workspace/tsyp_ws/src

RUN cp /opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf \
        /workspace/tsyp_ws/src/tsyp_simulation/models/waffle_writer.sdf && \
    cp /opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf \
        /workspace/tsyp_ws/src/tsyp_simulation/models/waffle_executor.sdf && \
    sed -i \
        -e 's#<topic>cmd_vel</topic>#<topic>/model/writer_robot/cmd_vel</topic>#' \
        -e 's#<odom_topic>odom</odom_topic>#<odom_topic>/model/writer_robot/odom</odom_topic>#' \
        -e 's#<topic>imu</topic>#<topic>/model/writer_robot/imu</topic>#' \
        -e '0,/<topic>scan<\/topic>/s#<topic>scan</topic>#<topic>/model/writer_robot/scan</topic>#' \
        /workspace/tsyp_ws/src/tsyp_simulation/models/waffle_writer.sdf && \
    sed -i \
        -e 's#<topic>cmd_vel</topic>#<topic>/model/executor_robot/cmd_vel</topic>#' \
        -e 's#<odom_topic>odom</odom_topic>#<odom_topic>/model/executor_robot/odom</odom_topic>#' \
        -e 's#<topic>imu</topic>#<topic>/model/executor_robot/imu</topic>#' \
        -e '0,/<topic>scan<\/topic>/s#<topic>scan</topic>#<topic>/model/executor_robot/scan</topic>#' \
        /workspace/tsyp_ws/src/tsyp_simulation/models/waffle_executor.sdf

RUN source /opt/ros/jazzy/setup.bash && \
    rosdep update --rosdistro jazzy && \
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y && \
    colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
