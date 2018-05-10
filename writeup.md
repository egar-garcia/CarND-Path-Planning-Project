**Path Planning Project**


Self-Driving Car Engineer Nanodegree Program

In this project, your goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.


### Build

This project involves the Term 3 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

```
mkdir build
cd build
cmake ..
make
./path_planning
```


## [Rubric](https://review.udacity.com/#!/projects/318/rubric) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Compilation

#### The code compiles correctly.

##### Code must compile without errors with ```cmake``` and ```make```.
###### Given that we've made ```CMakeLists.txt``` as general as possible, it's recommend that you do not change it unless you can guarantee that your changes will still compile on any platform.

As the file ```CMakeLists.txt``` was made as general as possible (including necessary steps for compiling in Mac, which is the platform I used to work in this project) I left if unchanged. Thus, after setting up [uWebSocketIO](https://github.com/uWebSockets/uWebSockets), the compilation and building can be done using the standard procedure (from the project's top directory):

```
mkdir build
cd build
cmake ..
make
./path_planning
```


### Valid Trajectories

#### The car is able to drive at least 4.32 miles without incident..
##### The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.

Please take a look at the following videos, taken from a screen recording from my laptop running the path planning program and the term 3 simulator. They show some runs, all of them running more than 4.32 miles without incidents.

[![Path Planning Run 1](http://img.youtube.com/vi/kKj7H9uCjGk/0.jpg)](http://www.youtube.com/watch?v=kKj7H9uCjGk)

[![Path Planning Run 2](http://img.youtube.com/vi/vwO0Win4kyk/0.jpg)](http://www.youtube.com/watch?v=vwO0Win4kyk)

[![Path Planning Run 3](http://img.youtube.com/vi/AoMb0vCxg-A/0.jpg)](http://www.youtube.com/watch?v=AoMb0vCxg-A)

#### The car drives according to the speed limit.
##### The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.

In order to drive as far as possible, but bellow the speed limit and avoiding to crash with the vehicle in front, a method method to adjust the speed was generated (see lines 375-386 of ```path_planner.cpp```), which according to the behavioral state chosen, reduces or increases the speed of the car, limited bellow by a minimum (0.001 m/s) and an above by an ideal velocity. The ideal velocity is set to the maximum minus 1 (to have some error range), in this case the ideal velocity is 49 MHP (converted to m/s) based in a maximum of 50 MPH (see line 27 of ```path_planner.cpp```).

#### Max Acceleration and Jerk are not Exceeded.
##### The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

In order to not exceeding the acceleration, a maximum value of 10 m/s^2 was set, and similarly as the speed and ideal acceleration is used, in this case half of the maximum acceleration to have a range of error (see line 28 of ```path_planner.cpp```). In the same method to adjust the speed the ideal acceleration plays an important role, since the speed is incremented or decremented using its value (see lines 375-386 of ```path_planner.cpp```), the speed is adjusted by adding the acceleration multiplied by the time between each car movement (20 milliseconds).

The strategy to avoid exceeding the jerk (10 m/s^3) was the one suggested in the walkthrough, which is using the previous path points that have not being consumed, in order to use them as a base to extend the next points of the path, and using a spline to smooth as much as possible the transitions between car positions.

#### Car does not have collisions.
##### The car must not come into contact with any of the other cars on the road.

As part as the development of a solution a method called ```scanSurroundings()``` was created to check the positions of the other cars on the highway (see lines 226-303 of ```path_planner.cpp```). This method identifies if a car is too close and if there are gaps in the left and right lanes to make a safe lane change. Gaps are identified by considering a safe distance to the cars in front and behind.

In order the establish a safe distance, the typical driving method of considering the distance in terms of time was used (because the actual distance depends on the speed). The recommended value in real life is 3 seconds (or more if driving conditions are difficult), but for this project an amount of 2 seconds was chosen to make a little bit more dynamic for visual purposes.

#### The car stays in its lane, except for the time between changing lanes.
##### The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

In order to stay in a lane, as mentioned in class, the transformation from Frenet coordinates to cartesian coordinates was used. The routes are calculated based in a constant value of the ```d``` component with distances ahead using the component ```s``` (see lines 176-191 of ```path_planner.cpp```), then transformed to cartesian coordinates that at the end are used to fill a spline which generates a smooth path in terms of cartesian coordinates (see lines 193-224 of ```path_planner.cpp```).

To do a lane change, the value of ```d``` is adjusted by the width of the lane (4 m) for the points ahead in the path, then the spline is in charge of generating a smooth path (see lines 67-74 of ```path_planner.cpp```). A variable called ```action_seconds``` (see line 110 of ```path_planner.h```) was generated to define the the time that a steep in the path is going to take, in particular a change of lane. The value selected was 2 seconds, which accomplishes the goal of making the change of lane in less than 3 seconds.  

#### The car is able to change lanes
##### The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

Using the previously mentioned scanning method to check the positions of other cars, some information is retrieved which describes if a car in front is to close, if there are gaps in the adjacent lanes and information about the cars in front on the current and adjacent lanes (see lines 226-303 of ```path_planner.cpp```).

Using this scanning information, the possible behavioral next states are generated, for the purpose of this project we have 4 possible states (see lines 22-35 of ```path_planner.h```):
- **Advance**: Increment speed till reaching the ideal.
- **Reduce Speed**: Reducing speed because a vehicle in front is too close or the maximum speed is being reached.
- **Change to Left Lane**: If there is a left lane and there is a gap in it.
- **Change to Right Lane**: If there is a right lane and there is a gap in it.

The next state is picked according with a cost function (see lines 328-340 of ```path_planner.cpp```), that basically considers the more speed the lest cost (of course respecting the maximum speed limit).

If the next state picked is a change of lane, most probably because there is a slow car in front and they are gaps in either of the lanes, then a change of lane is done as described in the previous point, i.e. using a spline for generating the trajectory with an adjusted value of the Frenet coordinate ```d``` for some distance ahead (see lines 64-90 of ```path_planner.cpp```).


### Reflection

#### There is a reflection on how to generate paths.
##### The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation".

The reflection is being writing in this same document. Most of the steps and mechanism were described in the previous points, but here comes the extended discussion.

Path generation is implemented in the method ```updatePath()``` of the class ```PathPlanner``` (lines 64-90 of ```path_planner.cpp```).

In general there would be some remaining previous path points which have not being consumed, the last one is used to determine the car's position and velocity in the future (for this project a path 2 seconds ahead was chosen). At the beginning the values of the current position are used. This described position is going to be known as *planned position*, similarly for velocity *planned velocity*.

As a result of the method to scan the positions of the other cars, the next possible behavioral states are generated, evaluated, and the one with the minor cost is picked.

The next part of the path is generated based on the Frenet coordinates, distance intervals are taken form the planned position of the car and adding increments to the component ```s```, these increments are given by the planned velocity of the car and the time taken by an action. In case of a change of lane an adjusted value of ```d``` is used in the values of ```s``` ahead of the planned position of the car, see the method ```fillPathPoints()``` of ```PathPlanner``` (lines 176-191 of ```path_planner.cpp```). These points are converted into to cartesian coordinates.

Using the next points previously obtained, the planned position is taken as base to translate and rotate them, with the purpose of the planned position of the car ends at the cartesian point (0, 0) and yaw 0, which simplifies the math (see lines 198-207 of ```path_planner.cpp```).

The transformed points are used to fill a a spline (see lines 198-207 of ```path_planner.cpp```). For splines the well known "simple cubic spline interpolation" library is used (downloaded and included in file ```spline.h```).

Once the spline is obtained, the next points in the path are generated by increasing the value of the ```x``` coordinate (this is valid since we are considering the car at (0, 0) with yaw 0), the increments on ```x``` are calculated according to the speed and acceleration required by the next behavioral state. Then finally the new points are rotated and transformed back to the original car's planned position (see lines 212-223 of ```path_planner.cpp```) and added after the previous points that have not being consumed.
