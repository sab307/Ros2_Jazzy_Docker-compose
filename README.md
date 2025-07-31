# ROS2 Jazzy Docker-compose.yml
First run the following commands accoridngly.
```
git clone this-repo
git submodule init
git submodule update
source /opt/ros/jazzy/setup.bash
sudo apt update
rosdep install -i -y --from-paths src --rosdistro jazzy
colcon build
```

Should run this in the docker container.

P.S.
```
sudo apt-get install net-tools && apt-get install iputils-ping
```