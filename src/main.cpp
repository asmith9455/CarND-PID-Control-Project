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

enum class SearchState
{
  RIGHT_BOUND_SEARCH,
  LEFT_BOUND_SEARCH,
  BETWEEN_BOUNDS,
  COMPLETE
};

static const std::map<SearchState, std::string> SEARCH_STATE_TO_STRING
{
  {SearchState::RIGHT_BOUND_SEARCH, "RIGHT_BOUND_SEARCH"},
  {SearchState::LEFT_BOUND_SEARCH, "LEFT_BOUND_SEARCH"},
  {SearchState::BETWEEN_BOUNDS, "BETWEEN_BOUNDS"},
  {SearchState::COMPLETE, "COMPLETE"}
};

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
          static std::vector<double> search_points_between_bounds;
          static std::vector<double> errors_between_bounds;
          static int search_between_bounds_counter{0};
          static int number_of_between_bounds_searches{0};
          static int between_bounds_searches_counter{0};
          static SearchState state = SearchState::RIGHT_BOUND_SEARCH;
          static SearchState prev_state = SearchState::RIGHT_BOUND_SEARCH;

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
            std::cout << "p before: " << s_pid_p << std::endl;
            std::cout << "testing error history: " << std::endl;
            for (const double &error : testing_error_history)
              std::cout << error << ", ";
            std::cout << std::endl;
            std::cout << "parameter history: " << std::endl;
            for (const double &p : parameter_history)
              std::cout << p << ", ";
            std::cout << std::endl;

            bool complete{false};

            while (!complete)
            {

              std::cout << "left bound is: " << max_param_left << std::endl;
              std::cout << "right bound is: " << max_param_right << std::endl;

              std::cout << "processing state: " << SEARCH_STATE_TO_STRING.at(state) << std::endl;
              if (state == SearchState::RIGHT_BOUND_SEARCH)
              {
                explore_direction = 1.0;
                if (error_increased_on_the_last_iteration)
                {
                  std::cout << "finished search for right bound - reversing search direction" << std::endl;
                  state = SearchState::LEFT_BOUND_SEARCH;
                  max_param_right = parameter_history.back();
                  complete = false;
                }
                else
                {
                  std::cout << "accelerating search for right bound" << std::endl;
                  explore_factor *= 1.1;
                  s_pid_p = s_pid_p + explore_direction * explore_factor;
                  complete = true;
                }
                prev_state = SearchState::RIGHT_BOUND_SEARCH;
              }
              else if (state == SearchState::LEFT_BOUND_SEARCH)
              {
                explore_direction = -1.0;
                std::cout << "error_increased_on_the_last_iteration: "<< error_increased_on_the_last_iteration << std::endl;
                std::cout << "prev_state: "<< SEARCH_STATE_TO_STRING.at(prev_state) << std::endl;
                if (error_increased_on_the_last_iteration && prev_state == SearchState::LEFT_BOUND_SEARCH)
                {
                  std::cout << "finished search for left bound - starting binary search between bounds" << std::endl;
                  state = SearchState::BETWEEN_BOUNDS;
                  max_param_left = parameter_history.back();
                  complete = false;
                }
                else
                {
                  std::cout << "accelerating search for left bound" << std::endl;
                  explore_factor *= 1.1;
                  s_pid_p = s_pid_p + explore_direction * explore_factor;
                  complete = true;
                }

                prev_state = SearchState::LEFT_BOUND_SEARCH;
              }
              else if (state == SearchState::BETWEEN_BOUNDS)
              {
                if (prev_state == SearchState::LEFT_BOUND_SEARCH)
                {
                  between_bounds_searches_counter = 1;
                }

                if (search_points_between_bounds.empty())
                {
                  search_points_between_bounds.push_back(max_param_left + 0.00 * (max_param_right - max_param_left));
                  search_points_between_bounds.push_back(max_param_left + 0.25 * (max_param_right - max_param_left));
                  search_points_between_bounds.push_back(max_param_left + 0.50 * (max_param_right - max_param_left));
                  search_points_between_bounds.push_back(max_param_left + 0.75 * (max_param_right - max_param_left));
                  search_points_between_bounds.push_back(max_param_left + 1.00 * (max_param_right - max_param_left));
                  for(const auto& search_point : search_points_between_bounds)
                  {
                    std::cout << "added search point: " << search_point << std::endl;
                  }
                  search_between_bounds_counter = 0;
                }
                else
                {
                  std::cout << "moving to next point, appending latest error test" << std::endl;
                  errors_between_bounds.push_back(testing_error_history.back());
                  ++search_between_bounds_counter;
                }

                if (search_between_bounds_counter == 5)
                {
                  std::cout << "finished testing all points" << std::endl;
                  
                  bool found_increase_coming_from_the_left{false};
                  bool found_increase_coming_from_the_right{false};

                  for(int i = 0; i < search_points_between_bounds.size() - 1; ++i)
                  {
                    if (errors_between_bounds[i+1] > errors_between_bounds[i])
                    {
                      found_increase_coming_from_the_left = true;
                      max_param_left = search_points_between_bounds[std::max(i-1, 0)];
                      std::cout << errors_between_bounds[++i] << ">" << errors_between_bounds[i] << " so adjusting max_param_left to: " << max_param_left << std::endl;
                      break;
                    }
                  }

                  for(int i = search_points_between_bounds.size() - 1; i >= 1; --i)
                  {
                    if (errors_between_bounds[i-1] > errors_between_bounds[i])
                    {
                      found_increase_coming_from_the_right = true;
                      max_param_right = search_points_between_bounds[std::min(i+1, static_cast<int>(search_points_between_bounds.size() - 1))];
                      std::cout << errors_between_bounds[i-1] << ">" << errors_between_bounds[i] << " so adjusting max_param_right to: " << max_param_right << std::endl;
                      break;
                    }
                  }

                  if(!found_increase_coming_from_the_left)
                  {
                    //there must only be one point on the right side of the minimum
                    std::cout << "found no parameter increase coming from the left - setting to second last point since there must be only one point on the right side of the minimum" << std::endl;
                    max_param_left = search_points_between_bounds[search_points_between_bounds.size() - 2];
                  }

                  if(!found_increase_coming_from_the_right)
                  {
                    //there must only be one point on the left side of the minimum
                    std::cout << "found no parameter increase coming from the right - setting to second point since there must be only one point on the left side of the minimum" << std::endl;
                    max_param_right = search_points_between_bounds[1];
                  } 

                  complete = false;
                  ++between_bounds_searches_counter;
                  if(between_bounds_searches_counter == number_of_between_bounds_searches)
                  {
                    const auto it = std::min_element(errors_between_bounds.cbegin(), errors_between_bounds.cend());
                    s_pid_p = search_points_between_bounds[it - errors_between_bounds.begin()];
                    state = SearchState::COMPLETE;
                  }

                  search_points_between_bounds.clear();
                  errors_between_bounds.clear();
                  search_between_bounds_counter = 0;
                }
                else
                {
                  std::cout << "setting search point to: " << search_points_between_bounds[search_between_bounds_counter] << std::endl;
                  s_pid_p = search_points_between_bounds[search_between_bounds_counter];
                  complete = true;
                }
                prev_state = SearchState::BETWEEN_BOUNDS;
              }
              else if (state == SearchState::COMPLETE)
              {
                complete = true;
                prev_state = SearchState::COMPLETE;
              }
              else
              {
                throw std::logic_error("invalid state");
              }

              std::cout << "explore_direction: " << explore_direction << std::endl;
              std::cout << "explore_factor: " << explore_factor << std::endl;
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