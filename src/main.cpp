//#ifndef DEBUG
//#define DEBUG 0
//#endif

#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
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

int main(int argc, char** argv) {
  uWS::Hub h;

  PID pid;
  bool steering_twiddle_enabled = false;
  bool throttle_twiddle_enabled = false;
  /**
   * Initialize the pid variable.
   */
  #ifdef DEBUG
    std::cout << "Debugging enabled..." << std::endl;
  #endif

  std::cout << "Initialize steering pid" << std::endl;
  if(argc > 1) {
    pid.Init(atof(argv[1]), atof(argv[2]), atof(argv[3]));
  } else {
    pid.Init(0.2, 0.004, 3.0);
  }

  if(argc >4) {
    if(atoi(argv[4]) == 1) {
      std::cout << "Steering Twiddle enabled" << std::endl;
      steering_twiddle_enabled = true;
    }
  }

  if(argc >5) {
    if(atoi(argv[5]) == 1) {
      std::cout << "Throttle Twiddle enabled" << std::endl;
      throttle_twiddle_enabled = true;
    }
  }

  PID pid_throttle;
  std::cout << "Initialize throttle pid" << std::endl;
  pid_throttle.Init(0.9, 0, 2.87);

  h.onMessage([&pid, &pid_throttle, &steering_twiddle_enabled, &throttle_twiddle_enabled](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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

          #ifdef DEBUG
          double speed = std::stod(j[1]["speed"].get<string>());
//          double angle = std::stod(j[1]["steering_angle"].get<string>());
          #endif

          double steer_value;
          /**
           * Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          double throttle;
          pid.UpdateError(cte);

          // Get steering value
          steer_value = pid.TotalError();

          // Apply twiddle to steering pid
          if(steering_twiddle_enabled) {
            pid.Twiddle(0.1);
          }

          /**
           * Set the raw steering value as the throttle's 'cross track error'.
           * By using the steering value, the throttle output can be calculated to be be inversely proportional
           * to the steering value and rate of change of the steering value.
           * The integral component is ignored as it is assumed that there is no throttle bias
           */
          // Throttle depends on the steering value, rate of change of steering value
          pid_throttle.UpdateError(pid.TotalError());
          throttle = pid_throttle.ThrottleOutput();

          // Apply twiddle to throttle pid
          if(throttle_twiddle_enabled) {
            pid_throttle.Twiddle();
          }

          // DEBUG
          #ifdef DEBUG
          double average_squared_err = pid.AverageSquaredError();

          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << " Counter: " << pid.counter << " Avg Err: " << average_squared_err
                    << " Speed: " << speed << " Throttle: " << throttle
                    << std::endl;
          #endif
          
          if(pid.counter == 150) { ws.close(); }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
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