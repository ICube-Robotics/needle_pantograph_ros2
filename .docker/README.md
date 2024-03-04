# EtherCAT Driver ROS2 Docker Containers
Provides a basic preconfigured docker container for development purposes.
To use it, make sure you have [Docker](https://docs.docker.com/get-docker/) installed, then build and run the image :

```shell
$ docker build --tag needle_pantograph_ros2:humble --file .docker/Dockerfile .
$ docker run -it --privileged --network=host needle_pantograph_ros2:humble
```
