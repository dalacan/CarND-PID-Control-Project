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

bool pid_debug = false;

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

  /**
   * Preset steering PID values
   */
  double steering_p = 0.347387;
  double steering_i = 0.000059049;
  double steering_d = 4.93047;

  bool steering_twiddle_enabled = false;
  double steering_twiddle_tol = 0.2;
  double steering_twiddle_batch_size = 50;
  double steering_dp_p = 0.1;
  double steering_dp_i = 0.0001;
  double steering_dp_d = 1;

  /**
   * Preset throttle PID values
   */
  double throttle_p = 0.9;
  double throttle_i = 0;
  double throttle_d = 2.87;

  bool throttle_twiddle_enabled = false;
  double throttle_twiddle_tol = 0.2;
  double throttle_twiddle_batch_size = 50;
  double throttle_dp_p = 0.1;
  double throttle_dp_i = 0.0001;
  double throttle_dp_d = 1;

  /**
   * Preset throttle limits
   */
  double min_throttle = 0.1;
  double max_throttle = 0.3;

  string help_text = "Usage\n\n pid "
                     "[options]\n\n"
                     "Options\n"
                     "-s <p> <i> <d>                                       = Set Steering PID values (-s 0.3 0.0001 3)\n"
                     "-t <p> <i> <d>                                       = Set Throttle PID values (-t 0.9 0 2)\n"
                     "-ws <tolerance> <batch_size>                         = Enable twiddle for steering and set tolerance \n"
                     "                                                       and batch size (-ws 0.2 50)\n"
                     "-ws <tolerance> <batch_size> <p_mod> <i_mod> <d_mod> = Enable twiddle for steering and set tolerance,\n"
                     "                                                       batch size and pid modifiers (-ws 0.2 50 0.1 0.0001 1)\n"
                     "-wt <tolerance> <batch_size>                         = Enable twiddle for throttle and set tolerance \n"
                     "                                                       and batch size (-wt 0.2 50)\n"
                     "-wt <tolerance> <batch_size> <p_mod> <i_mod> <d_mod> = Enable twiddle for throttle and set tolerance,\n"
                     "                                                       batch size and pid modifiers (-wt 0.2 50 0.1 0.0001 1)\n"
                     "-l <min_throttle> <max_throttle>                     = Set minimum and maximum throttle (-l 0.1 0.3)\n"
                     "-d                                                   = Debug output enabled\n"
                     "-h                                                   = Displays this help menu\n";

  /**
   * Initialize the pid variable.
   */
  // Handle application input
  if(argc > 1) {
    for (int i = 1; i < argc; i++) {
      if (argv[i][0] == '-') {
        switch (argv[i][1]) {
          case 's':
            std::cout << "Steering PID set" << std::endl;
            steering_p = atof(argv[i + 1]);
            steering_i = atof(argv[i + 2]);
            steering_d = atof(argv[i + 3]);
            break;
          case 't':
            std::cout << "Throttle PID set" << std::endl;
            throttle_p = atof(argv[i + 1]);
            throttle_i = atof(argv[i + 2]);
            throttle_d = atof(argv[i + 3]);
            break;
          case 'w':
            switch (argv[i][2]) {
              case 's':
                if (atof(argv[i + 1]) > 0) {
                  std::cout << "Steering twiddle" << std::endl;
                  steering_twiddle_enabled = true;
                  steering_twiddle_tol = atof(argv[i + 1]);
                  steering_twiddle_batch_size = atoi(argv[i + 2]);

                  if ((i + 3) < argc && argv[i + 3][0] != '-') {
                    std::cout << "Steering twiddle modifiers" << std::endl;
                    steering_dp_p = atof(argv[i + 3]);
                    steering_dp_i = atof(argv[i + 4]);
                    steering_dp_d = atof(argv[i + 5]);
                  }
                }
                break;
              case 't':
                if (atof(argv[i + 1]) > 0) {
                  std::cout << "Throttle twiddle" << std::endl;
                  throttle_twiddle_enabled = true;
                  throttle_twiddle_tol = atof(argv[i + 1]);
                  throttle_twiddle_batch_size = atoi(argv[i + 2]);

                  if ((i + 3) <argc && argv[i + 3][0] != '-') {
                    std::cout << "Throttle twiddle modifiers" << std::endl;
                    throttle_dp_p = atof(argv[i + 3]);
                    throttle_dp_i = atof(argv[i + 4]);
                    throttle_dp_d = atof(argv[i + 5]);
                  }
                }
            }
            break;
          case 'l':
            std::cout << "Throttle limits" << std::endl;
            min_throttle = atof(argv[i + 1]);
            max_throttle = atof(argv[i + 2]);
            break;
          case 'd':
            std::cout << "Debug enabled" << std::endl;
            pid_debug = true;
            break;
          case 'h':
              std::cout << help_text;
              return 0;
          default:
            break;
        }
      }
    }
  }

  std::cout << "Initialize steering pid" << std::endl;
  pid.Init(steering_p, steering_i, steering_d);

  if(steering_twiddle_enabled) {
    pid.InitTwiddle(steering_twiddle_tol, steering_twiddle_batch_size, steering_dp_p, steering_dp_i, steering_dp_d);
  }

  PID pid_throttle;
  std::cout << "Initialize throttle pid" << std::endl;
  pid_throttle.Init(throttle_p, throttle_i, throttle_d);
  pid_throttle.InitThrottle(min_throttle, max_throttle);

  if(throttle_twiddle_enabled) {
    pid.InitTwiddle(throttle_twiddle_tol, throttle_twiddle_batch_size, throttle_dp_p, throttle_dp_i, throttle_dp_d);
  }

  h.onMessage([&pid, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          double speed;
//          double angle;
          if(pid_debug) {
            speed = std::stod(j[1]["speed"].get<string>());
//            angle = std::stod(j[1]["steering_angle"].get<string>());
          }

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

          // Apply twiddle to steering pid if enabled
          pid.Twiddle();

          /**
           * Set the raw steering value as the throttle's 'cross track error'.
           * By using the steering value, the throttle output can be calculated to be be inversely proportional
           * to the steering value and rate of change of the steering value.
           * The integral component is ignored as it is assumed that there is no throttle bias
           */
          // Throttle depends on the steering value, rate of change of steering value
          pid_throttle.UpdateError(pid.TotalError());
          throttle = pid_throttle.ThrottleOutput();

          // Apply twiddle to throttle pid if enabled
          pid_throttle.Twiddle();

          // DEBUG
          if(pid_debug) {
            double average_squared_err = pid.AverageSquaredError();

            std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                      << " Counter: " << pid.counter << " Avg Err: " << average_squared_err
                      << " Speed: " << speed << " Throttle: " << throttle
                      << std::endl;
          }

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