#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

//#define TWIDDLE   // Twiddle optimiser
#define SPEED 30.0  // Target speed for simple 'P' controller.

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

  PID pid;
  // TODO: Initialize the pid variable.

/*
  // PID settings based on recommendations from 'pid_control_document.pdf' at http://georgegillard.com/documents.
  pid.Init(0.10, 0.0000, 0.00); // 1) Long increasing oscillations - does not make it to first corner.
  pid.Init(0.20, 0.0000, 0.00); // 2) Faster oscillations leaving track earlier.
  pid.Init(0.40, 0.0000, 0.00); // 3) Even faster oscillations leaving track earlier - acrobatic style!
  pid.Init(0.10, 0.0000, 1.00); // 4) Low P from above, plus D - first complete track circuit!
  pid.Init(0.10, 0.0000, 2.00); // 5) Increased D - improved Average CTE from step above!
  pid.Init(0.10, 0.0000, 3.00); // 6) Increased D - slightly better Average CTE from step above!
  pid.Init(0.10, 0.0010, 3.00); // 7) Add I - more 'jumpy' but Average CTE is improved.
  pid.Init(0.10, 0.0030, 3.00); // 8) Increase I - Average CTE worse.
  pid.Init(0.10, 0.0020, 3.00); // 9) Decrease I - Average CTE closer to 7).
  pid.Init(0.10, 0.0015, 3.00); // 10) Average I from 7) and 9) - Average CTE closer to 7).
*/

/*
  // Final selection with best Average CTE across the track used for Twiddle input.
  pid.Init(0.10, 0.0010, 3.00);
*/
#ifdef TWIDDLE
  pid.Twiddle(0.05, 0.0001, 0.3, 150, 1650);
#endif

  // Final Twiddle parameters.
  pid.Init(0.458088, 0.00119765, 4.65089);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          pid.UpdateError(cte);
          steer_value = fmax(-1.0, fmin(1.0, pid.TotalError()));

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = fmax(0.0, fmin(0.3, (SPEED-speed)*0.15));
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

#ifdef TWIDDLE
          if (pid.TwiddleReset()) {
            std::string msg = "42[\"reset\", {}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
#endif
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
