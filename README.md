# Simple ROS Publisher and Subscriber
A simple implementation of ros publisher and subscriber

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

## Dependencies
- Ubuntu 20.0 or above
- ROS2 Humble/Foxy

## License
```
MIT License

Copyright (c) 2022 Shailesh Pranav Rajendran

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Build
- Create a workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
- Clone the repository
```
git clone https://github.com/shaileshpranav/beginner_tutorials.git
```
- Build the workspace
```
cd ~/ros2_ws/src
colcon build --packages-select beginner_tutorials
cd .. && . install/setup.bash
```
## Run
- Run the Publisher node
```
ros2 run beginner_tutorials talker
```
- In another terminal, run the Subscriber node
```
ros2 run beginner_tutorials listener
```

## Run using Launch file
- Change frequency value to desired
```
ros2 launch beginner_tutorials pubsub.yaml freq:=1
```

## Change String by calling Service
- To change the Published string, run the talker node (Server)
```
ros2 run beginner_tutorials talker
```
- The Published string can be changed by calling the service (Client) and passing the new string in the argument as follows
```
ros2 run beginner_tutorials serv "Terps_Rocks"
```