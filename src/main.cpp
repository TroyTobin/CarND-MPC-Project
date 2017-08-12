#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "util.hpp"

// for convenience
using json = nlohmann::json;

#define LATENCY_MS  (100)

// Order of polynomial to fit
#define POLY_ORDER    (4)

/**
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 */
string hasData(string s) 
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) 
  {
    return "";
  } 
  else if (b1 != string::npos && b2 != string::npos) 
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() 
{
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, 
                     char *data, 
                     size_t length,
                     uWS::OpCode opCode) 
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') 
    {
      string s = hasData(sdata);
      if (s != "") 
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") 
        { 
          // Waypoints
          vector<double> waypoints_x_global = j[1]["ptsx"];
          vector<double> waypoints_y_global = j[1]["ptsy"];

          // Position
          double pos_x_global  = j[1]["x"];
          double pos_y_global  = j[1]["y"];

          // Orientation
          double psi = j[1]["psi"];

          // Velocity
          double velocity = j[1]["speed"];

          // Steering Angle
          double steering_angle_cur = j[1]["steering_angle"];

          // sanity check sizes of vectors
          assert(waypoints_x_global.size() == waypoints_y_global.size());

          // From the "Tips and Tricks"
          // Remember that the server returns waypoints using the map's coordinate system, 
          // which is different than the car's coordinate system. 
          // Transforming these waypoints will make it easier to both display them and to 
          // calculate the CTE and Epsi values for the model predictive controller.

          // Transform the waypoints to the vehicle coordinates
          Eigen::VectorXd waypoints_x_veh(waypoints_x_global.size());
          Eigen::VectorXd waypoints_y_veh(waypoints_y_global.size());
          for (int i = 0; i < waypoints_x_global.size(); i++)
          {
            // Translation equation from http://planning.cs.uiuc.edu/node99.html
            double waypoint_x_veh = ((double)waypoints_x_global[i] - pos_x_global) * cos(-psi) - 
                                        ((double)waypoints_y_global[i] - pos_y_global) * sin(-psi);

            double waypoint_y_veh = ((double)waypoints_x_global[i] - pos_x_global) * sin(-psi) + 
                                        ((double)waypoints_y_global[i] - pos_y_global) * cos(-psi);

            waypoints_x_veh[i] = waypoint_x_veh;
            waypoints_y_veh[i] = waypoint_y_veh;
          }

          /*******************************************************
           *
           *  Allow for latency by forward projecting the vehicle
           *
           *******************************************************/

          // vehicle orientation in vehicle coordinates
          // From the "tips and tricks" psi_t+1 = psi_t - (v_t/L_f) * steering_angle * delta_time
          double orientation_veh =  - (velocity/Lf) * steering_angle_cur * LATENCY_MS/1000;

          // vehicle position in vehicle coordinates
          double pos_x_veh = (velocity * LATENCY_MS/1000) * cos(steering_angle_cur);
          double pos_y_veh = (velocity * LATENCY_MS/1000) * sin(steering_angle_cur);


          // Fit a polynomial to the waypoints in vehicle coordinates
          auto coeffs = polyfit(waypoints_x_veh, waypoints_y_veh, POLY_ORDER);

          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.
          double cte = polyeval(coeffs, pos_x_veh) - pos_y_veh;

          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          double epsi = orientation_veh - atan(polyeval_derivative(coeffs, pos_x_veh));

          Eigen::VectorXd state(6);
          state << pos_x_veh, pos_y_veh, orientation_veh, velocity, cte, epsi;

          std::vector<double> model_x_vals = {};
          std::vector<double> model_y_vals = {};
          double steering_angle = 0.0;
          double acceleration   = 0.0;

          auto vars = mpc.Solve(state, coeffs);

          // Extract the data from the model
          // Steering angle
          steering_angle = vars[0];

          // Acceleration
          acceleration = vars[1];

          // Model projection
          for (int v = 2; v < vars.size() - 1; v++)
          {
            model_x_vals.push_back(vars[v]);
            model_y_vals.push_back(vars[v+1]);
            v++;
          }

          json msgJson;
          
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steering_angle / DEG2RAD(25);
          msgJson["throttle"] = acceleration;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals = model_x_vals;
          vector<double> mpc_y_vals = model_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = model_x_vals;
          msgJson["mpc_y"] = model_y_vals;

          // Display the waypoints/reference line
          vector<double> next_waypoints_x_vals;
          vector<double> next_waypoints_y_vals;

          for (int s = 0; s < waypoints_x_veh.size(); s++)
          {
            next_waypoints_x_vals.push_back(waypoints_x_veh[s]);
            next_waypoints_y_vals.push_back(waypoints_y_veh[s]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_waypoints_x_vals;
          msgJson["next_y"] = next_waypoints_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(LATENCY_MS));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
