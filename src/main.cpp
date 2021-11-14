#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

//helper funtions from helper.h
// hasDate - 
// distance - Calculates distance between two points 
// closestWaypoint - indentifies closest waypoint from maps of waypoints on the highway
// NextWaypoint - next waypoint in the front, might not be the closest 
// getFrenet - transform from x,y to s,d
// getXY - inverse of getFrenet


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  //define lane
  int lane = 1;
          
  //define reference target velocity in mph
  double ref_val = 0; // setting initial ref speed to zero to avoid initial jerk at cold start 
  if(ref_val < 49.5)
  {
    ref_val += 0.224; //initial accelerate by 5m/s2
  }  

  h.onMessage([&ref_val, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // previous based on last set of points that car follows before running through this current set of points 
          int prev_size = previous_path_x.size();
          
          //code for avoiding collision with car in front 
          //simulator is noting x,v,vx,vy of all the cars on the road
          // we use this information to predict where the other cars are at, how fast they are going and how we should behave as a result 
          
          //partial code is taken from Udacity Q&A video 
          if(prev_size > 0)
          {
            car_s = end_path_s; //previous path's last location s 
          }
          
          
          bool too_close = false;
          bool car_same_lane = false;
          bool car_left_lane = false;
          bool car_right_lane = false;
          double check_speed;
          
          //find ref_v to use 
          for (int i =0; i < sensor_fusion.size(); i++) //go through sensor fusion list 
          {
            //determine if car is in my lane 
            float d = sensor_fusion[i][6];// sensor_fusion[i] is data about ith car and its 6th vaalue which is d
            
            // determine other cars' lane
            int car_lane = -1;
            if (d>0 && d<4)
            {
              car_lane =0;
            } 
            else if (d>4 && d<8)
            {
              car_lane=1;
            }
            else if (d>8 && d<12)
            {
              car_lane = 2;
            }
            
            //every car's speed irrespective of its lane 
            double vx = sensor_fusion[i][3];//check speed of the car if its in my lane 
            double vy = sensor_fusion[i][4];
            check_speed = sqrt(vx*vx+vy*vy); // speed of the car. helpful for predicting where the car will be in future 
            double check_car_s = sensor_fusion[i][5]; //check car's s value to see if it is close to use. 
            
            check_car_s +=((double)prev_size*0.02*check_speed); //if using previous points, we can project s value outwards in time 
            // because if we are using previous path points, our car is still little behind of where it is supposed to be so we want to see what the car will look like in future 
            
            
            // check if the front car in the lane is too close        
            if(d<(2+4*lane+2) && d>(2+4*lane-2)) //condition for other car in my lane 
            { 
              // check if our car's s is closer to other car's s 
              if ((check_car_s > car_s)&&((check_car_s-car_s) <30)) // if car is within 30m and in front
              {
                //lower reference velocity so that we dont crash cars in front of us 
                //we could also flag the car to change lanes - implement this 
                too_close = true;      
              }
            }
            //detect car in the left lane 
            if (d<(2+4*(lane-1)+2) && d>(2+4*(lane-1)-2)) 
            {
              if (abs(check_car_s - car_s) < 30)
              {
                car_left_lane = true;
              }
            }
            
            //detect car in right lane 
            if (d<(2+4*(lane+1)+2) && d>(2+4*(lane+1)-2)) 
            {
              if (abs(check_car_s - car_s) < 30)
              {
                car_right_lane = true;
              }
            }
          }
          //if car in my current lane is too close in the front
          
          if(too_close)
          {
            if(car_right_lane == false && lane<2)
            {
              lane++;
            }
            else if(car_left_lane == false && lane>0)
            {
              lane--;
            }
            else 
            {
              ref_val-=0.25;
            }
    
          }
          
          else if(ref_val < 49.5)
          {
            ref_val += 0.224; //accelerate by 5m/s2
          }          
          
          //list of widely spaced waypoints at 30m to use spline function on and fill in more points 
          vector<double> ptsx;
          vector<double>ptsy;
          
          //reference x,y,yaw states 
          // starting point could either be current state of car or previous path's endpoint
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          //if previous size is empty, we use car as starting point 
          if (prev_size<2)
          {
            //use two points that make go tangent to the car 
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);   
            
          }
          else
          {
            //redefine reference state as previous path end points 
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
            
            //use two points that make tangent to previous paths endpoints 
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);    
            
          }
          
          //addint 30m spaced points in frenet distance on the road ahead of starting reference that we just calculated 
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);  
          
          //at this point ptsx, ptsy has 5 points in it 2+3
          
          //shift car's reference point to origin, for math simplicity 
          for (int i =0; i<ptsx.size();i++)
          {
            //shift car reference angle to zero degree 
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            
          }
          
          //create a spline 
          tk::spline s;
          
          //set x,y points to the spline 
          s.set_points(ptsx,ptsy);
          
          //define the actual (x,y) points to be used for the path planner 
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //start with all the previous path points from the last time 
          for(int i=0; i < previous_path_x.size();i++)
          {
            next_x_vals.push_back(previous_path_x[i]); //actual points that path planner will use 
            next_y_vals.push_back(previous_path_y[i]);
          }
          
         // calculate how to break up spline points so that we travel at our desired reference speed 
          double target_x =30.0; //horizon x value 
          double target_y = s(target_x); //this calculates corresponding y for every n point
       
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          double x_add_on =0;
          
          //fill up the rest of the path planner after filling it with previous points. here we will always output 50 points 
          for (int i =1; i<= 50-previous_path_x.size();i++)
          {
            double N = (target_dist/(0.02*ref_val/2.24)); // mph--> 2.24 m/s
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point; 
            
            //rotate back to normal after rotating it earlier to car reference 
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point+=ref_x;
            y_point+=ref_y;
           
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          

          //END

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}