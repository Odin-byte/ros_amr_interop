FROM osrf/ros:humble-desktop-full

WORKDIR /dev_ws/

RUN apt update && apt install ros-humble-ament* mosquitto-clients -y 
# && rm -rf /var/lib/apt/lists/*

ADD vda5050_msgs /dev_ws/src/vda5050_msgs
ADD vda5050_serializer /dev_ws/src/vda5050_serializer
ADD vda5050_connector /dev_ws/src/vda5050_connector
ADD vda5050_bringup /dev_ws/src/vda5050_bringup

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths src -y --ignore-src && colcon build --symlink-install"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /dev_ws/install/local_setup.bash" >> ~/.bashrc

# When inside the docker image, run ros2 run vda5050_connector mqtt_bridge.py and ros2 run vda5050_connector vda5050_controller.py
# Or create a bringup file that does all of this, including the adaptor

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

# Build with eg. `docker build . -t inorbit`
# Then execute with `docker run -it inorbit "/bin/bash -c \"ros2 launch vda5050_bringup vda5050_launch.py\""`