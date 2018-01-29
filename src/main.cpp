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
#include <ctime>
#include <time.h>

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
  cout << "test" << endl;
  //measuring how frequently the car receives telemetery data
  auto timestamp0 = std::chrono::high_resolution_clock::now();
  auto timestamp1 = std::chrono::high_resolution_clock::now();
  int n = 0; //counting telemetry messages
  double cum_time = 0; // cum sum of time elapsed
  int k = 10;
  vector<double> dts_prev(k);
  vector<double> dts_curr(k);
  for(int i = 0; i < k; i++){
    dts_prev[i] = 0;
    dts_curr[i] = 0;
  }
  
  h.onMessage([&mpc, &timestamp0, &timestamp1, &n, &cum_time, &dts_prev, &dts_curr](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          
          /*MEASURE TIME FROM MESSAGE TO MESSAGE
          timestamp1 = std::chrono::high_resolution_clock::now();
          n +=1;
          for (int i = dts_curr.size()-2; i>=0; i-- ){
            dts_curr[i] = dts_prev[i+1];
          }
          dts_curr[dts_curr.size() - 1] = std::chrono::duration<double, std::milli>(timestamp1 - timestamp0).count() /1000.;
          double dts_sum = 0;
          for (int i = 0; i < dts_curr.size();i++){
            dts_sum += dts_curr[i];
          }

          double dt = 0.001;
          if (n < dts_curr.size()){
            dt = dts_sum / (n * 1.);
          }
          else {
            dt = dts_sum / (dts_curr.size() * 1.);
          }
          dts_prev = dts_curr;           
          cout << "dt " << dt << endl;                
          timestamp0 = std::chrono::high_resolution_clock::now();*/

          //if (n >1000) {
          //  return 0;
          //}

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"]; // 6 datapoints
          vector<double> ptsy = j[1]["ptsy"]; // 6 datapoints
          double px = j[1]["x"]; 
          double py = j[1]["y"]; 
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          //convert to vehicle coordinates
          Eigen::VectorXd eptsx_vehicle(ptsx.size());
          Eigen::VectorXd eptsy_vehicle(ptsy.size());
          for (int i = 0; i < ptsx.size(); i++){
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            eptsx_vehicle[i](dx * cos(-psi) - dy * sin(-psi));
            eptsy_vehicle[i](dx * sin(-psi) + dy * cos(-psi));
          } 
          

          
          //approximate target x and y values for the space in between waypoints 
          auto coeffs = polyfit(eptsx_vehicle, eptsy_vehicle, 3);
          std::cout << "coeffs" << std::endl;
          std::cout << "c0: " <<coeffs[0] << ",c1: "<<coeffs[1] << ",c2: "<< coeffs[2] <<",c3: "<< coeffs[3] << std::endl;
          double cte = polyeval(coeffs, 0) - 0;
          std::cout << "cte" << std::endl;
          std::cout << cte << std::endl;
          // desired psi is a derivative of polynomial f(x) at x:
          // for polynomial of order 3:
          // f(x) = a*x^3 + b*x^2 + c*x + d, so
          // f'(x) = 3*a*x^2 + 2*b*x + c
          int x_direction = 0;
          if (ptsx[0] > ptsx[1]){
            x_direction = 1;
          }

          double epsi = psi - (atan(coeffs[1] + 2 * coeffs[2] * px  + 3 * coeffs[3] * px * px) + x_direction * M_PI); 
          std::cout << "epsi" << std::endl;
          std::cout << epsi << std::endl;

          Eigen::VectorXd current_state(6);
          current_state << px, py, psi, v, cte, epsi;
          auto solution = mpc.Solve(current_state, coeffs, x_direction, dt);
          
          double steer_value;
          double throttle_value;

          steer_value = solution[0]/deg2rad(25);
          throttle_value = solution[1];
          std::cout << "steer_value" << std::endl;
          std::cout << steer_value << std::endl;
          std::cout << "throttle_value" << std::endl;
          std::cout << throttle_value << std::endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          //convert to vehicle's coordinates
          for( int i = 0; i < ptsx.size(); i++) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            next_x_vals.push_back(dx * cos(-psi) - dy * sin(-psi));
            next_y_vals.push_back(dx * sin(-psi) + dy * cos(-psi));
          }          

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          //this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
      timestamp0 = timestamp1;
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
