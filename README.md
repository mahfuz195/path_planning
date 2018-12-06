# CarND Path Planning Project
Self-Driving Car Engineer Nanodegree Program

### Goals
The goal of this project is to build a path planner that creates smooth, safe trajectories for the car to follow. The highway track has other vehicles, all going different speeds, but approximately obeying the 50 MPH speed limit.

The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.

#### Path Planning Constraints:
1. The car has to travel atleast 4.32 miles without any collisions.

2. The car must not go above the 50 mph speed limit.

3. The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

4. The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

5. The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

#### Given higway map and wayponint data

Inside data/highway_map.csv there is a list of waypoints that go all the way around the track. The track contains a total of 181 waypoints, with the last waypoint mapping back around to the first. The waypoints are in the middle of the double-yellow dividing line in the center of the highway.

The track is 6945.554 meters around (about 4.32 miles). If the car averages near 50 MPH, then it should take a little more than 5 minutes for it to go all the way around the highway.

The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right-hand side. The car should always be inside a lane unless doing a lane change.

Each waypoint has an (x,y) global map position, and a Frenet s value and Frenet d unit normal vector (split up into the x component, and the y component). 

```
[x,y,s,dx,dy]
```

The s value is the distance along the direction of the road. The first waypoint has an s value of 0 because it is the starting point.

The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road. The d vector can be used to calculate lane positions. For example, if you want to be in the left lane at some waypoint just add the waypoint's (x,y) coordinates with the d vector multiplied by 2. Since the lane is 4 m wide, the middle of the left lane (the lane closest to the double-yellow dividing line) is 2 m from the waypoint. 


### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).


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

## Reflection

For generating the trajectory, we have used a spline library for smooth trajectory. As the spline takes atleast 3 input points. I have genegarted three previous points using the trigonometry formula. The ```vector<double> ptsx``` and ```vector<double> ptsy``` has been used to keep track of the previous points of the car's trajectory. The code is as follows in main.cpp:
```
 if(prev_size <2)
 {
      double prev_car_x = car_x - cos(car_yaw);
      double prev_car_y = car_y - sin(car_yaw);

      ptsx.push_back(prev_car_x);
      ptsx.push_back(car_x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(car_y);
}
```
Then we take three waypoints to generate the car future trajectory. The calcuation is done in Frenet corordinate and lated converted it to (x,y) co-ordinate:
```
vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
```
Then we perform a spline function to generate smooth trajectory using the following lines of code, where target y is the smooth curve of the trajectory:
```
tk::spline s;
s.set_points(ptsx, ptsy);
...
double target_y = s(target_x);  

double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
double x_add_on = 0;

// Fill up the rest of the path planner to always output 50 points
for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
  double N = (target_dist/(.02*ref_vel/2.24));
  double x_point = x_add_on + (target_x) / N;
  double y_point = s(x_point);

  x_add_on = x_point;

  double x_ref = x_point;
  double y_ref = y_point;

  // Rotate back to normal after rotating it earlier
  x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
  y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

  x_point += ref_x;
  y_point += ref_y;

  next_x_vals.push_back(x_point);
  next_y_vals.push_back(y_point);
}
```

### Lane change

I have implemented a simple logic for lane changing. The logic is a follows:

1. The car will keep driving in the same lane, if there is no front vehicle.

2. If there is a vehcile in-front of the vehcile, it will check the left and right lane and if they are avaiable, the car will change its lane to the free lane.

### Conclusion
The car was able to successfully drive the entire track (4.31miles) without any incidents. There are several improvements that can be performed in the project. It could make car a better choice btween light and left lane, based on the traffic condition of each lane such as average speed of each lane and can compute a short term cost benefit calculation. 




  




