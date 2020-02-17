# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Goal of the project
In this project it is aimed to navigate safely on a highway with other vehicles. The main purpose is driving safely without exceeding the speed limit and without any collision. It is desired for car to be driven as close as possible to the 50 MPH speed limit. The ego vehicle can overtake slower vehicles and adjust its speed according to traffic conditions.

## Code Review
### Defined Variables

["lane"] The car's current lane (0: left lane, 1: middle lane; 2: right lane)

["ref_vel"] The car's reference velocity 

["targetLane "] The lane which is suitable for overtaking

["too_close"] Flag to whether clarify if there is a car in front of the vehicle

["left_lane_change_possible"] Flag to whether clarify if there is a car on the left lane of the current lane

["right_lane_change_possible"] Flag to whether clarify if there is a car on the right lane of the current lane

["safe_distance_1"] Safe distance of changing to right lane

["safe_distance_2"] Safe distance of changing to left lane

["accMax"] Max. acceleration

### Code Explanation
This project uses the codes which is taken from the Q&A video that is provided by Udacity. Lots of concepts are taken from this code.

#### Lane Change Algorithm: 

##### Initialize Lane & Reference Velocity (Line: 202-205): 
Current lane and the reference velocity is initialized.

##### State and Safe Distance Definition (Line: 248-264):
3 states (too_cloose, left_lane_change_possible, right_lane_change_possible) are defined.
Safe distances are defined to clarify states. By using these information, future behaviours are planned.
Two distances are defined in order to change lanes. Left lane change is easier to right lane change because the safety distance to change left is 5 meters shorter than other distance. As an approximation, overtaking to left lane is prioritized. 


##### Lane Definition (Line: 266-292):
The variable d which is the lateral position in terms of frenet coordinates is taken as input. Based on the lateral position interval, the lanes are defined. (For e.g. left lane: 0 < d <4 meters) Current lane of the vehicle is shown on the terminal as messages.

##### Lane Change Preparation (Line: 302-326):
At first, if the ego vehicle is located in defined lane, it checks if there is a car in front of it compared to safe distance intervals. Additionally, other lanes are checked individually based on defined safe distance intervals for right and left lane changes.
According to the comparison, flags that indicates future behaviours are defined. 

##### Behavior Execution (Line: 328-356):
When the flag conditions are satisfied, required actions are executed. If the too_close condition is satisfied and left lane change is possible, assign lane to left adjacent lane (except the ego vehicle is in the left lane.). If it is not satisfied, right lane change is executed (except the ego vehicle is in the right lane). If there is no lane change possibilities, and obstacle exists in front of the vehicle, it decelerates.  For acceleration, deceleration, accMax is defined as 0.2 in order not to exceed acceleration and jerk limits. If current lane is free, and there is no lane change possibilities, the ego vehicle accelerates.

##### Trajectory Calculation (Line: 359-478):
The ideas and concepts are taken from the Q&A video. Trajectories are calculated with the help of splines based on the speed and lane output from the behavior, car coordinates and past path points.


## Conclusion
The car is able to:
-Drive according to the speed limit.
-Drive without exceeding jerk and acceleration limits.
-Overtake speed up and speed down when is necessary
-Navigate without collision

An output can be seen in [Output.png](/home/CarND-Path-Planning-Project/src/Output.png)


## FunFact.
-Changing between manual and automated driving may cause some unexpected outputs such as flying self-driving car.
See [FunOutput.png](/home/CarND-Path-Planning-Project/src/Funfact.png)


-------------------------------------------------------- 0 --------------------------------------------------------


Udacity's original README for the project repo

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab] (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).




