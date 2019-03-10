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
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));
      // std::cout << "s is: " << s << std::endl;

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value, throttle;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          // steering controller

          static int experiment_counter{0};
          static const int experiment_counter_limit{100};
          static double total_error_for_last_cycle{0.0};
          static std::vector<double> testing_error_history{};
          // static std::vector<bool> switch_direction_history{};
          static double explore_direction{1.0};
          static double explore_factor{1.0};
          static bool first_time{true};

          static auto s_pid_p{0.1};
          static const auto s_pid_d{0.5};
          static const auto s_pid_i{0.0};

          if (experiment_counter >= experiment_counter_limit)
          {
            experiment_counter = 0;
            testing_error_history.push_back(total_error_for_last_cycle);
            total_error_for_last_cycle = 0.0;

            bool error_increased_on_the_last_iteration{false};
            bool error_increased_on_the_last_2_iterations{false};

            if (switch_direction_history.size() >= 2)
            {
              const double error_1_back = testing_error_history[testing_error_history.size() - 1];
              const double error_2_back = testing_error_history[testing_error_history.size() - 2];
              error_increased_on_the_last_iteration = error_1_back > error_2_back;
              
              if (switch_direction_history.size() >= 3)
              {
                const double error_3_back = testing_error_history[switch_direction_history.size() - 3];
                const bool error_increased_on_the_last_2_iterations = error_increased_on_the_last_iteration && error_2_back > error_3_back;

                // const bool switch_1_back = switch_direction_history[switch_direction_history.size() - 1];
                // const bool switch_2_back = switch_direction_history[switch_direction_history.size() - 2];
                // const bool switched_last_2_iterations = switch_1_back && switch_2_back;

                // if (!switched_last_2_iterations)
                // {
                //   throw std::logic_error("There is a problem with the logic. If we had error increases the last 2 times, we should have switched directions both times.");
                // }
              }
            }

            if (error_has_increased_in_both_directions)
            {
              explore_factor *= 0.9;
            }

            if (error_has_increased)
            {
              // switch_direction_history.push_back(true);
              explore_direction = -1.0 * explore_direction;
            }
            else
            {
              // switch_direction_history.push_back(false);
              explore_factor *= 1.1;
            }

            s_pid_p = s_pid_p + explore_direction * explore_factor;
          }

          static double last_cte{cte};
          static double cte_sum{0.0};

          throttle = 0.3;

          steer_value = -s_pid_p * cte - s_pid_d * (cte - last_cte) - s_pid_i * cte_sum;

          last_cte = cte;
          cte_sum += cte;

          total_error_for_last_cycle += cte;

          ++experiment_counter;

          //speed controller

          double ref_speed = 10.0;
          double speed_error = ref_speed - speed;
          static double last_speed_error{speed_error};

          const auto t_pid_p{0.1};
          const auto t_pid_d{0.01};
          const auto t_pid_i{0.001};

          throttle = t_pid_p * speed_error + t_pid_d * (speed_error - last_speed_error);

          last_speed_error = speed_error;

          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value
          //           << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}