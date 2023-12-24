# Ground Station for Operations and Testing (GSOT)

## Overview

The GSOT repository offers a software platform ideal for remote data transmission and processing across various applications.

## Components

-   `/agent`: Simulates remote environment with capabilities to transmit data using ROS.

-   `/client`: Acts as the receiving and processing unit, equipped with data recievers and UI components.

## Getting Started

### Prerequisites

-   [Docker](https://www.docker.com/)

### Installation and Setup

1. Clone the repository and navigate to it.

2. To build and start the containers in detached mode, run the following command:

```sh
docker-compose up -d --build
```

3. Enter the `client` container:

```sh
docker exec -it client bash
```

4. Install node dependencies for the client:

```sh
cd server
npm install
cd ../frontend
npm install
```

5. Start the client by running the following command in the `frontend` directory:

```sh
npm run start-dev
```

6. In a new terminal instance, enter the `agent` container:

```sh
docker exec -it agent bash
```

7. Once inside, navigate to the ROS workspace and build the packages using `colcon`:

```sh
cd /root/workspace/ros2
source /opt/ros/foxy/setup.bash
colcon build
```

8. Launch the `pub_tester` ROS nodes after sourcing the newly built packages:

```sh
source /root/workspace/ros2/install/setup.bash
ros2 launch pub_tester pub_tester.launch.xml
```

9. In a new terminal instance, launch the `gsot` ROS nodes:

```sh
docker exec -it agent bash
source /opt/ros/foxy/setup.bash
source /root/workspace/ros2/install/setup.bash
ros2 launch gsot gsot.launch.xml
```

10. Visit `http://localhost:3000` to interact with the client.
