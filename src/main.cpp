#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <fstream>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid_steer;

  pid_steer.Init(0.185,  0.0005,  0.0025, true); // @param (Kp_, Ki_, Kd_)

  PID pid_velocity;

  pid_velocity.Init(0.01,  0.000015,  0.00, false); // @param (Kp_, Ki_, Kd_)

  std::ofstream steering_pid_log;
  std::ofstream speed_pid_log;

  steering_pid_log.open("steering_pid_run.csv");
  speed_pid_log.open("speed_pid_run.csv");


  double max_speed = 20;
  double target_speed = 0;

  double wait_after_spawn_counter = 8;


  /**
   * TODO: Initialize the pid variable.
   */

  h.onMessage([&pid_steer, &pid_velocity, &steering_pid_log, &speed_pid_log, &target_speed, &max_speed, &wait_after_spawn_counter](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {


	  double total_error = 0;
	  double throttle_pos = 0;


    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          

          if(wait_after_spawn_counter>0)
          {
        	  steer_value = 0;
        	  throttle_pos = 0;
        	  wait_after_spawn_counter -= 1;
          }
          else
          {
        	   // DEBUG
			  std::cout << "CTE: " << cte << " Steering Value: " << steer_value
						<< std::endl;

			  pid_steer.UpdateError(cte);

			  total_error = pid_steer.TotalError();

			  total_error = std::max(-1.0, std::min(total_error, 1.0)); // https://stackoverflow.com/questions/9323903/most-efficient-elegant-way-to-clip-a-number clipping a variable

			  steering_pid_log << cte << "," << total_error << std::endl;

			  //std::cout << "PID steer error: " << total_error << std::endl;

			  steer_value =  -total_error;

			  //target_speed = (max_speed * (1 - sqrt(steer_value*steer_value)));
			  target_speed = max_speed; 

			  //std::cout << " -------------- target_speed: " << target_speed << "Steering value: " << steer_value <<" scaler: " << (1 - sqrt(steer_value*steer_value)) << std::endl;


			  double speed_error = target_speed - speed;
			  pid_velocity.UpdateError(speed_error);

			  throttle_pos = pid_velocity.TotalError();

			  //std::cout << "PID total throttle pos: " << throttle_pos << std::endl;


			  throttle_pos = std::max(-1.0, std::min(throttle_pos, 1.0));

			  std::cout << throttle_pos << std::endl;
			  //std::cout << "PID throttle pos: " << throttle_pos << std::endl;

			  speed_pid_log << speed_error << "," << throttle_pos << std::endl;
          }







          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_pos;





          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
