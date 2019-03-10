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
          static std::vector<double> parameter_history{};
          // static std::vector<bool> switch_direction_history{};
          static double explore_direction{1.0};
          static double max_param_right{std::numeric_limits<double>::max()};
          static double max_param_left{-std::numeric_limits<double>::max()};
          static double explore_factor{1.0};
          static bool first_time{true};

          static bool hit_threshold_on_last_iter{false};

          //optimization starting point
          static auto s_pid_p{0.1};
          static const auto s_pid_d{0.5};
          static const auto s_pid_i{0.0};

          //implement local minimization algorithm
          if (experiment_counter >= experiment_counter_limit)
          {
            experiment_counter = 0;
            testing_error_history.push_back(total_error_for_last_cycle);
            parameter_history.push_back(s_pid_p);
            total_error_for_last_cycle = 0.0;

            bool error_increased_on_the_last_iteration{false};
            bool error_increased_on_the_last_2_iterations{false};

            if (testing_error_history.size() >= 2)
            {
              const double error_1_back = testing_error_history[testing_error_history.size() - 1];
              const double error_2_back = testing_error_history[testing_error_history.size() - 2];
              error_increased_on_the_last_iteration = error_1_back > error_2_back;

              if (testing_error_history.size() >= 3)
              {
                const double error_3_back = testing_error_history[testing_error_history.size() - 3];
                error_increased_on_the_last_2_iterations = error_increased_on_the_last_iteration && error_2_back > error_3_back;
              }
            }

            std::cout << "\n\n"
                      << std::endl;
            std::cout << "------------------------------------------" << std::endl;
            std::cout << "------------------------------------------" << std::endl;
            std::cout << "finished experiment " << testing_error_history.size() << std::endl;
            std::cout << "testing error history: " << std::endl;
            for (const double &error : testing_error_history)
              std::cout << error << ", ";
            std::cout << std::endl;
            std::cout << "parameter history: " << std::endl;
            for (const double &p : parameter_history)
              std::cout << p << ", ";
            std::cout << std::endl;
            std::cout << "error_increased_on_the_last_iteration: " << error_increased_on_the_last_iteration << std::endl;
            std::cout << "error_increased_on_the_last_2_iterations: " << error_increased_on_the_last_2_iterations << std::endl;
            std::cout << "hit_threshold_on_last_iter: " << hit_threshold_on_last_iter << std::endl;
            std::cout << "max_param_right: " << max_param_right << std::endl;
            std::cout << "max_param_left: " << max_param_left << std::endl;

            // logic below is based on the assumption that this is a local minimization problem

            if (error_increased_on_the_last_2_iterations || hit_threshold_on_last_iter)
            {
              // we have overshot the solution in both directions
              explore_factor *= -0.9;
              std::cout << "we have overshot the solution in both directions" << std::endl;
            }
            else if (error_increased_on_the_last_iteration)
            {
              // we have overshot the solution in the current direction
              if (explore_direction == 1.0)
              {
                std::cout << "setting max_param_right to " << s_pid_p << std::endl;
                max_param_right = s_pid_p;
              }
              else if (explore_direction == -1.0)
              {
                std::cout << "setting max_param_left to " << s_pid_p << std::endl;
                max_param_left = s_pid_p;
              }
              else
              {
                throw std::logic_error("logic error");
              }

              explore_direction = -1.0 * explore_direction;
              std::cout << "we have overshot the solution in the current direction only - let's try the other one" << std::endl;
            }
            else
            {
              //we should search faster in the current direction
              explore_factor *= 1.1;
              std::cout << "we need to search faster in the current direction" << std::endl;
            }

            const auto change_in_parameter = explore_direction * explore_factor;

            std::cout << "explore_direction: " << explore_direction << std::endl;
            std::cout << "explore_factor: " << explore_factor << std::endl;
            std::cout << "change for p: " << change_in_parameter << std::endl;
            std::cout << "p before: " << s_pid_p << std::endl;

            s_pid_p = s_pid_p + change_in_parameter;

            hit_threshold_on_last_iter = false;

            if (s_pid_p > max_param_right)
            {
              s_pid_p = max_param_right;
              hit_threshold_on_last_iter = true;
            }
            
            if (s_pid_p < max_param_left)
            {
              s_pid_p = max_param_left;
              hit_threshold_on_last_iter = true;
            }

            std::cout << "p after: " << s_pid_p << std::endl;
          }

          static double last_cte{cte};
          static double cte_sum{0.0};

          throttle = 0.3;

          steer_value = -s_pid_p * cte - s_pid_d * (cte - last_cte) - s_pid_i * cte_sum;

          last_cte = cte;
          cte_sum += cte;

          total_error_for_last_cycle += std::fabs(cte);

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
          // std::cout << msg << std::endl;
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