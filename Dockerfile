FROM osrf/ros:foxy-desktop

WORKDIR /dev_ws/

RUN apt update && apt install ros-foxy-ament* mosquitto-clients python3-paho-mqtt python3-pytest python3-pytest-mock -y
# && rm -rf /var/lib/apt/lists/*

ADD luka_adapter /dev_ws/src/luka_adapter
ADD vda5050_msgs /dev_ws/src/vda5050_msgs
ADD vda5050_serializer /dev_ws/src/vda5050_serializer
ADD vda5050_connector /dev_ws/src/vda5050_connector
ADD vda5050_bringup /dev_ws/src/vda5050_bringup
ADD move_base_msgs /dev_ws/src/move_base_msgs

# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && rosdep install --from-paths src -y --ignore-src"
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --symlink-install"


RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "source /dev_ws/install/local_setup.bash" >> ~/.bashrc

# When inside the docker image, run ros2 run vda5050_connector mqtt_bridge.py and ros2 run vda5050_connector vda5050_controller.py
# Or create a bringup file that does all of this, including the adaptor

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2 launch vda5050_bringup vda5050_launch.py"]

# Build with eg. `docker build . -t inorbit`
# Then execute with `docker run -it inorbit"` and the vda5050_bringup will be launch automatically