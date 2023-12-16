# Ground Station Software

## Overview

This repository is the development hub for our Ground Station Software. The repository is divided into two primary components:

- `/sub`: Simulates the Sub 2.0 environment, including a ROS network and data transmitters that send metric and camera data.

- `/gs`: Represents the ground station, equipped with data recievers, processing capabilities, and a user interface.

## Getting Started

1. Clone the repository and navigate to it.

2. To build and start the containers in detached mode, run the following command:

```sh
docker-compose up -d --build
```

3. Enter the `transmitter` container:

```sh
docker exec -it transmitter bash
```

4. Once inside, navigate to the ROS workspace:

```sh
cd /root/workspace/ros2
```

5. Source the ROS environment and build the packages using `colcon`:

```sh
source /opt/ros/foxy/setup.bash
colcon build
```

6. Source the newly built environment _before_ running anything:

```sh
source install/setup.bash
```

7. To test the transmitter, launch the `sub_pub_tester` nodes:

```sh
ros2 launch sub_pub_tester sub_pub_tester.launch.xml
```

8. Prepare for data transmission by entering the `reciever` container:

```sh
docker exec -it receiver bash
```

9. Once inside the `reciever` container, navigate to the workspace and run the following python scripts:

```sh
cd /root/workspace
python3 metric_data_reciever.py
python3 camera_data_reciever.py
```

**Note:** When running the `camera_data_receiver` script, be aware that displaying video streams through Docker requires additional setup for GUI forwarding.

10. Finally, in a new terminal session, enter the `transmitter` container again and launch the `sub_transmitter` nodes (make sure to source the environments):

```sh
docker exec -it transmitter bash
source /opt/ros/foxy/setup.bash
source /root/workspace/ros2/install/setup.bash
ros2 launch sub_data_transmission sub_data_transmission.launch.xml
```

11. To stop the containers run

```sh
docker-compose down
```
