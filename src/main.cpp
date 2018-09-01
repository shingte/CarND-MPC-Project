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

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          double delta= j[1]["steering_angle"];
          double a = j[1]["throttle"]; // acceleration

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // Convert global coordinates to car coordinates
          // - Shift the coordinates of ptsx and ptsy to origin of car
          // - Rotate the coordinates of ptsx and ptsy to be w.r.t. psi of car
          // Polyfit accepts Eigen vector and ptsx and ptsy are std::vectors. Make the conversion
          Eigen::VectorXd ptsx_car(ptsx.size());
          ptsx_car.fill(0.0);
          Eigen::VectorXd ptsy_car(ptsy.size());
          ptsy_car.fill(0.0);

          double dx;
          double dy;
          for (size_t i = 0; i < ptsx.size(); i++) {
            dx = ptsx[i] - px; //px_next;
            dy = ptsy[i] - py; //py_next;

            ptsx_car[i] = dx * cos(-psi) - dy * sin(-psi);
            ptsy_car[i] = dx * sin(-psi) + dy * cos(-psi);
          }
          auto coeffs = polyfit(ptsx_car, ptsy_car, 3);
         
          // Initial state.
          const double x = 0;
          const double y = 0;
          const double psi0 = 0;
          //Calculate cte and epsi. cte is the horizontal line
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          // Add latency into the model. 
          // State after delay.
          double delay = 0.1; //Actuator delay (latency) set to 0.1 sec (100 ms).
          double x_delay = x + ( v * cos(psi0) * delay );
          double y_delay = y + ( v * sin(psi0) * delay );
          double psi_delay = psi0 - ( v * delta * delay / Lf );
          double v_delay = v + a * delay;
          double cte_delay = cte + ( v * sin(epsi) * delay );
          double epsi_delay = epsi - ( v * atan(coeffs[1]) * delay / Lf );

          // Define the state vector.
          Eigen::VectorXd state(6);
          state << x_delay, y_delay, psi_delay, v_delay, cte_delay, epsi_delay;

          //Find the MPC solution.
          auto sol = mpc.Solve(state, coeffs);

          //Multiply steering angle by -1 as the implementation of positive, negative
          //angles and right, left turn in simulator is reversed in comparison to co-ordinate
          //system in vehicle's plane
          double steer_value = -sol[0]/deg2rad(25);
          double throttle_value = sol[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for ( size_t i = 2; i < sol.size(); i++ ) { 
            if ( i % 2 == 0 ) {
              mpc_x_vals.push_back( sol[i] );
            } else {
              mpc_y_vals.push_back( sol[i] );
            }
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

  
          for ( int i = 0; i < ptsx_car.size(); i++ ) {
            next_x_vals.push_back(ptsx_car[i]);
            next_y_vals.push_back(ptsy_car[i]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //auto msg = "42[\"steer\", steering_angle:"+std::to_string(steer_value)+"\nthrottle:"+std::to_string(throttle_value)+ "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
