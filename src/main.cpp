#include <math.h>
#include <uWS/uWS.h>
#include <algorithm>
#include <iostream>
#include <string>
#include "PID.h"
#include "json.hpp"

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
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main() {
    uWS::Hub h;

    bool isNFS = false;  // driving mode

    PID pid_steering, pid_throttle;
    double target_throttle;
    if (isNFS) {
        // racing mode
        pid_steering.Init(0.12, 8.33942e-05, 1.47849);
        pid_throttle.Init(0.126766, 9.32336e-05, 0.1137);
        target_throttle = 0.6;
    } else {
        // training mode - for steering turning

        // manual tuned: {0.1, 0.0001, 1.6}
        // twiddle tuned: {0.12 | 8.33942e-05 | 1.47849}
        pid_steering.Init(0.12, 8.33942e-05, 1.47849);

        // manual tuned: {0.1, 0.0001, 0.15}
        // twiddle tuned: {0.126766, 9.32336e-05, 0.1137}
        pid_throttle.Init(0., 0., 0.);
        target_throttle = 0.3;
    }

    h.onMessage([&pid_steering, &pid_throttle, &target_throttle](
                    uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                    uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message
        // event. The 4 signifies a websocket message The 2 signifies a
        // websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(string(data).substr(0, length));

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<string>());
                    double speed = std::stod(j[1]["speed"].get<string>());

                    bool isTwiddle_steering = false;
                    pid_steering.UpdateError(cte, isTwiddle_steering);
                    double steer_value = pid_steering.TotalError();

                    bool isTwiddle_throttle = false;
                    pid_throttle.UpdateError(cte, isTwiddle_throttle);
                    double throttle_value =
                        target_throttle + pid_throttle.TotalError();

                    // compansation for understeering when high speed
                    if (speed > 50) {
                        steer_value *= 1.04;
                    } else if (speed > 40) {
                        steer_value *= 1.02;
                    }

                    // the steering value is [-1, 1]
                    steer_value = std::max(std::min(steer_value, 1.), -1.);

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;
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
    });    // end h.onMessage

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