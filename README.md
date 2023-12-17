# Ground Station for Operations and Testing (GSOT)

## Overview

The GSOT repository offers a software platform ideal for remote data transmission and processing across various applications.

## Components

- `/agent`: Simulates remote environment with capabilities to transmit data using ROS.

- `/client`: Acts as the receiving and processing unit, equipped with data recievers and UI components.

## Getting Started

1. Clone the repository and navigate to it.

2. To build and start the containers in detached mode, run the following command:

```sh
docker-compose up -d --build
```

3. Enter the `agent` container:

```sh
docker exec -it agent bash
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

7. To test the transmitter, launch the `pub_tester` nodes:

```sh
ros2 launch pub_tester pub_tester.launch.xml
```

8. Prepare for data transmission by entering the `client` container:

```sh
docker exec -it client bash
```

9. Once inside the `client` container, navigate to the workspace and run the following python scripts:

```sh
cd /root/workspace
python3 metric_data_reciever.py
python3 camera_data_reciever.py
```

**Note:** When running the `camera_data_receiver` script, be aware that displaying video streams through Docker requires additional setup for GUI forwarding.

10. Finally, in a new terminal session, enter the `agent` container again and launch the `gsot` nodes (make sure to source the environments):

```sh
docker exec -it agent bash
source /opt/ros/foxy/setup.bash
source /root/workspace/ros2/install/setup.bash
ros2 launch gsot gsot.launch.xml
```

11. To stop the containers run

```sh
docker-compose down
```
