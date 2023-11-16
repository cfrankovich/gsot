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

3. To enter the `transmitter` container, use the following command:

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

6. Source the newly built environment before running anything:

```sh
source install/setup.bash
```

7. To test the transmitter by publishing messages to topics, you need to run the sub_pub_tester nodes:

```sh
ros2 run sub_pub_tester <node_name>
```

8. Prepare for data transmission by entering the `reciever` container:

```sh
docker exec -it receiver bash
```

9. Once inside the `reciever` container, navigate to the workspace and run the python script:

```sh
cd /root/workspace
python3 reciever.py
```

10. Finally, in a new terminal session, enter the `transmitter` container again and run the `sub_transmitter` (make sure to source the environments again):

```sh
docker exec -it transmitter bash
source /opt/ros/foxy/setup.bash
source /root/workspace/ros2/install/setup.bash
ros2 run sub_data_transmission metric_data_transmitter
```

11. To stop the containers run

```sh
docker-compose down
```
