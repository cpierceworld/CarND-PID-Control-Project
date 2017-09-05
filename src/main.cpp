#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "Twiddler.h"
#include <math.h>
#include <vector>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

bool do_twiddle = false;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID steer_pid;
  // TODO: Initialize the pid variable.
  //steer_pid.Init(0.15, 0.0000001, 1.5);
  steer_pid.Init(0.171, 0.0, 1.62);


  std::vector<PID> pidControllers;
  pidControllers.push_back(steer_pid);

  std::vector<double> dts;
  dts.push_back(0.01);
  dts.push_back(0.00000001);
  dts.push_back(0.1);

  Twiddler twiddler(0.00001, pidControllers, dts, "pid_coefficients.txt");

  int count = 0;
  h.onMessage([&steer_pid, &twiddler, &count](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          
          if(do_twiddle && count++ > 100) {
        	  //don't start doing twiddle till after 100 time steps

        	  twiddler.updateError(cte);

			  if(count > 800) {
				  // twiddle settings after 800 times steps, then start over
				  twiddler.twiddle();
				  steer_pid.p_error = 0;
				  steer_pid.i_error = 0;
				  steer_pid.d_error = 0;

				  count = 0;
				  string msg = "42[\"reset\",{}]";
				  std::cout << msg << std::endl;
				  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				  return;
			  }
          }

          // update error and calculate steer_value at each step
          steer_pid.UpdateError(cte);
          steer_value = -steer_pid.TotalError();
          if(steer_value < -1) steer_value = -1;
          else if(steer_value > 1) steer_value = 1;

          // slow up if turning too much
          throttle_value = 0.5 - fabs(steer_value * 1.75);
          if(speed < 10.0 && throttle_value < 0.1) {
        	  // don't let the car stop
        	  throttle_value = 0.1;
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << ", Throttle Value: "<< throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value; //0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
