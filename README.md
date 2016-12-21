## PX4: Towards a ROS 2.0 native drone ##

This project presents a *work in progress* prototype of the concept of a software autopilot for drones that speaks ROS 2.0 natively. The existing prototype served the purpose of validating the feasibility of having ROS 2.0 sitting at the core of a drone autopilot (the brain of the drone) and puts together a framework whereto start porting sensors (even if they’re virtual sensors such as the one in the prototype for the purposes of SITL) to ROS 2.0 which eventually will lead to deploy the prototype in a real, Linux-based autopilot similar to [Erle-Brain 3](http://erlerobotics.com/blog/erle-brain-3/).

The project has been conducted using [Erle-Copter](http://erlerobotics.com/blog/erle-copter/) Linux drone kit simulated in [Gazebo](gazebosim.org) simulator.

## Project status
- [x] SITL Barometer ported to ROS 2.0 (publication and subscription)
- [x] `systemcmd` tool for subscribing to ROS 2.0 topics
- [x] SITL GPS ported to ROS 2.0 (publication)
- [ ] SITL GPS ported to ROS 2.0 (subscription)
- [ ] SITL Gyroscope ported to ROS 2.0 (publication and subscription)
- [ ] SITL Accelerometer ported to ROS 2.0 (publication and subscription)
- [ ] SITL Gyroscope ported to ROS 2.0 (publication and subscription)
- [ ] SITL ADC ported to ROS 2.0 (publication and subscription)
- [ ] SITL Airspeed ported to ROS 2.0 (publication and subscription)
- ...

## How to contribute
- Select one of the unfinished tasks from above
- Open up an issue indicating the task you'll be working on
- Submit a PR and get it accepted

## Demo
A demo showing the barometer being published as a ROS 2.0 topic was demonstrated [here](https://www.youtube.com/watch?v=_eTzJZE5XV0&feature=youtu.be).

## How to reproduce
### Source your ROS 2.0 workspace
```
source ~/ros2_ws/install/setup.bash
```

### Simulate Erle-Copter
```
make posix_sitl_default gazebo_erlecopter

# now, within the PX4 prompt:
px4> commander takeoff # take off the drone
px4> commander land    # land the drone
```

Now we are able to listen in the sensor topic by just typing (in the prompt of px4):

```
px4> ros2_listener sensor_baro
```
