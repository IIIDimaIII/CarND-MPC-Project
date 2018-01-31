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
  //measuring how frequently the car receives telemetery data
  auto timestamp0 = std::chrono::high_resolution_clock::now();
  auto timestamp1 = std::chrono::high_resolution_clock::now();
  int n = 0; //counting telemetry messages
  double cum_time = 0; // cum sum of time elapsed
  int k = 10;
  vector<double> dv_prev(k);
  vector<double> dv_curr(k);
  for(int i = 0; i < k; i++){
    dv_prev[i] = 0;
    dv_curr[i] = 0;
  }
  double v_prev = 0;  
  
  h.onMessage([&mpc, &timestamp0, &timestamp1, &n, &cum_time, &dv_prev, &dv_curr, &v_prev](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          n +=1; 
          
          double dt = 0;
          
          //MEASURE TIME FROM PREVIOUS TELEMETRY MESSAGE
          timestamp1 = std::chrono::high_resolution_clock::now();
          
          
          dt = std::chrono::duration<double, std::milli>(timestamp1 - timestamp0).count() /1000.;
          cout << "dt " << std::chrono::duration<double, std::milli>(timestamp1 - timestamp0).count() /1000. << endl;                
          cum_time += std::chrono::duration<double, std::milli>(timestamp1 - timestamp0).count() /1000.;
          cout << "time elapsed " << cum_time << endl;                
          timestamp0 = std::chrono::high_resolution_clock::now();

          /*if (n >500) {
            return 0;
          }*/

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"]; // 6 datapoints
          vector<double> ptsy = j[1]["ptsy"]; // 6 datapoints
          double px = j[1]["x"]; 
          double py = j[1]["y"]; 
          double psi = j[1]["psi"];
          double v = j[1]["speed"] ;   
          v = v / 0.62137 * 1000./ 3600.; // convert to m/s
          //throttle to acceleration measurement
          /*for (int i = dv_curr.size()-2; i>=0; i-- ){
            dv_curr[i] = dv_prev[i+1];
          }
          dv_curr[dv_curr.size() - 1] = (v - v_prev) / dt;
          double dv_sum = 0;
          for (int i = 0; i < dv_curr.size();i++){
            dv_sum += dv_curr[i];
          }
          double average_a = 0;
          if (n < dv_curr.size()){
            average_a = dv_sum / (n * 1.);
          }
          else {
            average_a = dv_sum / (dv_curr.size() * 1.);
          }
          dv_prev = dv_curr;
          v_prev = v;
          cout << "v " << v << endl;
          cout << "average_a " << average_a << endl;*/
          
          //convert to vehicle coordinates
          Eigen::VectorXd eptsx_vehicle(ptsx.size());
          Eigen::VectorXd eptsy_vehicle(ptsy.size());
          for (int i = 0; i < ptsx.size(); i++){
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            eptsx_vehicle[i] = (dx * cos(-psi) - dy * sin(-psi));
            eptsy_vehicle[i] = (dx * sin(-psi) + dy * cos(-psi));
          } 
          
          
          //approximate target x and y values for the space in between waypoints 
          
          //auto coeffs = polyfit(eptsx_vehicle, eptsy_vehicle, 3);
          auto coeffs = polyfit(eptsx_vehicle, eptsy_vehicle, 2);          
          double cte = 0 - polyeval(coeffs, 0);
          
          // desired psi is a derivative of polynomial f(x) at x:
          // for polynomial of order 3:
          // f(x) = a*x^3 + b*x^2 + c*x + d, so
          // f'(x) = 3*a*x^2 + 2*b*x + c          
         
          //double epsi = 0 - (atan(coeffs[1] + 2 * coeffs[2] * 0  + 3 * coeffs[3] * 0 * 0)); 
          double epsi = 0 - (atan(coeffs[1] + 2 * coeffs[2] * 0)); 

          Eigen::VectorXd current_state(6);        
          current_state << 0, 0, 0, v, cte, epsi;          
          
          auto solution = mpc.Solve(current_state, coeffs);
          
          double steer_value = solution[0]/deg2rad(25);
          double throttle_value = solution[1];          

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          //msgJson["steering_angle"] = 0;
          //msgJson["throttle"] = 0.5;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for(int i = 2; i < solution.size(); i+=2){
            mpc_x_vals.push_back(solution[i]);
            mpc_y_vals.push_back(solution[i+1]);
          }

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
