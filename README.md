# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

----

## How it works
The path planner application was designed to work in tandem with Udacity's term 3 simulator which runs the path planning project. The application receives details from simulated sensors from the path planning project simulator and makes decisions to autonomously drive a vehicle around a highway track while keeping in a desired lane, avoiding other traffic, and making safe driving decisions to optimize travel. This process is done in three steps, namely: state prediction, behavior planning, and trajectory planning. 

#### State Prediction [(lines 297 - 347)](./src/main.cpp#L297)
State prediction is done by looping through each vehicle returned in the simulator's sensor fusion vector. For each vehicle, the current `s` value is updated to represent the vehicle's future `s'` state by adding in the product of the vehicle's speed, 0.02 (the time in seconds between simulator callbacks), and how many steps in the future we want to look. 
```
s' = s + (steps_ahead * 0.02 * speed)
```
Using this future state, in [lines 324-345](./src/main.cpp#L324), we compare to the other vehicles and fill state vectors to find the closest vehicles in front of and behind the ego vehicle in each lane to be used in the behavior planning portion of the path planner. 

It would be possible to improve this state prediction algorithm by implementing state over time tracking to get vehicle acceleration data from the time-series sensor results. This acceleration data could be used along with current speed in the `s'` calculation to better predict the future state of the vehicles of interest.

#### Behavior Planning [(lines 348 - 450)](./src/main.cpp#L348)
The behavior planning algorithm is implemented by doing cost base analysis on the different action options that are available to the ego vehicle. First, the alogorithm checks the forward vector for the ego's current lane to find if there is a vehicle in that lane within 30 meters in front of the ego vehicle, if the lane is free, the ego vehicle will stay in it's current lane and accelerate at a safe speed to a maximum of about 49.5 mph. If the lane is blocked, the algo iterates over three actions calculating the cost of taking those actions. These actions include, slow down and keep lane, change lane to the right, or change lane to the left. The main cost function applied to all three actions is the speed the ego vehicle would be able to achieve safely in the target lane after committing the action. This is done by multiplying the cost to completely stop the ego vehicle, set to 0.75, by the percent of the max speed we can travel in that lane.
```
cost = COST_STOP * ((MAX_VEL-forward[lane].speed)/MAX_VEL)
```
If it is not safe to change a lane to the left or right, for instance if a vehicle is within a buffer zone in front of or behind the ego in that lane or if making a change in that direction would leave the road, that actions cost is set to the max cost so that action is not taken.  

The alogrithm then loops over the calculated costs to find the lowest cost action. In case of a tie in actions, the choice is made by the order the costs are looped over because the choice is only changed if a cost is less than the current choices cost. The order this is done in is: keep same lane, change lane left, and finally change lane right.

With the best option calculated, the planner makes changes to the variables used by the trajectory planner to best follow the desired action.  In the case of keeping the same lane, the vehicle will change the desired ego velocity by reducing it until it matches the lead vehicles speed and has a buffer of at least 20 meters from the lead vehicle.  If the desired action is to change lanes the desired lane value is updated to initiat the lane change. Along with the lane change a step count is initiated to make sure that the ego vehicle will not initiate another lane change until the current one is complete.

#### Trajectory Planning [(lines 452 - 554)](./src/main.cpp#L452)

Trajectory planning was done using the spline library to calculate smooth transitions between waypoints. This is done by taking the last two points from the previously planned path and adding them as the first two points to be used in the spline plotter. Next, three new waypoints are added to the vector at 30, 60, and 90 meters out from the end of the vehicles currently planned path. These waypoints are calculated using the vehicles desired lane from the behavior planner along with the ego's last `s` value in it's previous path. These vectors are fed into the spline plotter to plot a smooth path between all of the points.

The planner then takes the currently planned path yet to be executed and appends new waypoints taken from the smoothed spline plotter to a max of 30 meters out from the vehicles current position to give a smoothed trajectory that follows the desired path.

----
   
### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

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

