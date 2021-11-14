# CarND-Path-Planning-Project - Highway Driving

## Overview 
The goal of this project is to design a path planning algorithm to drive car on a ~ 4-mile stretch of a highway on a Udacity simulator. The simulator sends position and speed coordinates of other cars on the road and in return, it expects a set of x,y points that represents car’s trajectory spaced at 0.02s
   
## Code Requirements 
* Code compilation without errors
* Car is able to drive at least 4.32 miles without errors – I tested the algorithm on the simulator for more than 5 miles and recorded no errors
* The car drives according to speed limit – Max speed limit of car is set to 49.5 mph as suggested in Q&A video. The car drives around that speed when no slow moving traffic is in the vicinity. When slow moving vehicle is in front, the car slows down or changes lane depending on the cars in adjacent lanes
* Max acceleration and jerk are not exceeded – Car is set to increase speed by 0.224mph as suggested in the Q&A video, this keeps the acceleration of the car around 5m/s2 which is below the max acceleration limit of 10m/s2. As a result, max acceleration and jerk limits are not exceeded throughout the drive.
* Car does not have collisions – Car does not have any collision with any other vehicle on the road because a logic for lane changing or slowdown is implemented in the code that is dependent on where are the cars in adjacent lanes located
* Car stays in the lane except for the time between changing lanes – car stays in the lane as long as there is no vehicle within 30m in the front. If the front vehicle is too close, then it looks for an opportunity to change lanes. If lane changing is not possible, then the car slows down until vehicle in front is at a safe distance.
* The car is able to change lanes – the car changes lanes if there is a vehicle in front of the car and it is safe to change lanes.

## Path Planning Algorithm
Path planning algorithm is implemented in the main.cpp from lines 114 to 339. Main logic to drive car on the highway while meeting criteria listed above is implemented in following steps
1. Cold start (lines 63-71) –
* Initialize current lane of the car (lane) as 1 and initial velocity (ref_val) as zero. Then slowly increase speed of the car by 0.224mph until its speed is lower than 49.5mph.
 ![7](https://user-images.githubusercontent.com/59345845/141700342-9171efac-a130-46a4-a7d5-cf4927d43577.JPG)

   ![1](https://user-images.githubusercontent.com/59345845/141699959-3d185ed7-b31d-4670-8b3c-8dfc8e5f0cfa.JPG)

2. Detecting cars in the all three lanes (lines 132-197)
* For every car in the sensor fusion list, define the car’s lane (car_lane) depending upon distance d from the center
      * 0<d<4 = car_lane, 0
      * 4<d<8 = car_lane, 1
      * 8<d<12 = car_lane, 2
* From the data in the sensor fusion list, extract x and y speed of the car (vx and vy), calculate absolute speed of every car and save it in variable check_speed. This is used in predicting where the car will be in future
* Save Frenet coordinate s for every car in check_car_s. This is used to check if the car is too close to us in the same lane or when changing lanes. If using previous points, we can project s value outward in time.
* In this section, we define three scenarios
      * If the car in the list is in my lane (this is identified based on the Frenet coordinate d of the car), and it is in front (this is done when check_car_s >car_s), and the distance is less than 30m. – then this car is too close to our car and we set the variable too_close to true.
      * If the car is in the left lane (identified based on d), and its within 30m of my car’s position in either front or rear (i.e. distance between my car and the car in left lane is less than 30m) – then this car_left_lane flag is set to true
      * If the car is in the right lane (identified based on d), and its within 30m of my car’s position in either front or rear (i.e. distance between my car and the car in left lane is less than 30m) – then this car_right_lane flag is set to true
      
![2](https://user-images.githubusercontent.com/59345845/141700084-2cb5027c-0a20-46fb-ac78-637a3b1627d2.JPG)


  ![3](https://user-images.githubusercontent.com/59345845/141700111-fa1e9088-21dd-406b-b170-bcd44c7c8858.JPG)


3. Actions based on car’s position –
The logic is set such that the car will only change its planned trajectory if the flag too_close is true in the earlier part of the code.
* If the car in front is too close in the current lane and there is not car within 30m in the right lane and the car is not in the rightmost lane – then the car is moved to adjacent right lane by incrementing lane by 1
* If the car in front is too close in the current lane and there is not car within 30m in the left lane and the car is not in the leftmost lane – then the car is moved to adjacent left lane by decrementing lane by 1
* If both the conditions are not true, then the car stays in the same lane but gradually decreases its speed so as to not collide with the car in front.
* Irrespective of the lane, when the car’s speed is less than 49.5mph reference speed, it is gradually increased so that the car drives at the recommended highway speed of 50mph for most part of the highway

![5](https://user-images.githubusercontent.com/59345845/141700152-a2438794-43e1-4336-b094-c3fcd044ac17.JPG)

4. Trajectory planning (lines 222 to 337)
* This part of the code forms a trajectory for the car to follow, using spline.h function, calculated lane and speed of the car calculated in the previous section of the code.
* As a starting position, we take two points that are tangent to the car’s current position or last two points from car’s previous trajectory along with three points 30m apart in the distance to create a spline.
* This calculation is performed by converting all Cartesian coordinates into car’s local
coordinates o Spline.h is used to calculate closely spaced points in these five points. These points are transformed back into Cartesian coordinates
* The speed change calculated in the previous part of the code is used in this section to increase or decrease the speed of the car on every trajectory points
* For a continuous trajectory, past trajectory points are continued to the new trajectory


![6](https://user-images.githubusercontent.com/59345845/141700189-71982478-4c59-4316-b3a7-56e6a87e89d7.JPG)


![7](https://user-images.githubusercontent.com/59345845/141700348-0496b0a4-cd3e-4a3d-8dc7-411fdc1ad08d.JPG)


![8](https://user-images.githubusercontent.com/59345845/141700354-6b4a74f7-04cb-4928-b0ae-8199b893562b.JPG)

