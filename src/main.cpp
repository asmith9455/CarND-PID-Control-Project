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

int main() {
  uWS::Hub h;
  

  /**
   * TODO: Initialize the pid variable.
   */

  h.onMessage([](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));
      // std::cout << "s is: " << s << std::endl;

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value, throttle;
          
          /// decription of effects:

          /// I started tuning P first, leaving the others set to 0. It seems that leaving P too low will not control the car forcefully enough, and it will not make the tight turns. Increasing P too far causes large oscillations that do not make for a smooth drive. Therefore, I chose P of 0.1 for steering to keep it on the track with noticeable oscillations. In the turns, the car goes very close to the outside of the track. Next I tuned the I and D parameters. Increasing D can reduce the oscillations, but making D larger first causes a more sluggish response (causing problems in the turns), then causes huge oscaillations that usually cause the car to leave the track. The I parameter is also tricky, because the sum of the cross track error can become quite large over time. Therefore, this parameter seems to typically be much smaller than the others. It is important to increase I in order to keep the car in the center of the track. P and D fight each other when the car is returning to center. This is because if the CTE is positive (negative) and the change in CTE is negative (positive), as it is when the car is returning to center, the P and D effects can counteract each other (they act in opposite directions). To move the car back to center, one can decrease D or increase P - both of these options will increase the amplitude and frequency of oscillations about the path the car should follow. Therefore, the effect of gently acting on accumulated error can move the car back to center. Note that in doing so, oscillations can still be caused because the sum of error does not immediately drop to 0, however it seems to work well enough to get the car around the track.  

          static PID steering_pid{0.1, 0.01, 4.0};

          static PID throttle_pid{-0.1, -0.0001, 0.0};

          steer_value = steering_pid.CalcNewError(cte, true);

          double ref_speed = 20.0;
          double speed_error = ref_speed - speed;

          throttle = throttle_pid.CalcNewError(speed_error);

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl << std::endl << std::endl;
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