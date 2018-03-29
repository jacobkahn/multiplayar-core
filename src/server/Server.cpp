#include "include/server/Server.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include "include/cv/SIFT.hpp"
#include "include/deps/crow.h"

const std::string kUserIDHeaderValue = "x-user-id";

Server::Server() {}

void Server::run(uint32_t port) {
  // Start main server thread
  app.port(port).multithreaded().run();
}

void Server::setup() {
  // Test endpoint
  CROW_ROUTE(app, "/test").methods("POST"_method)([](const crow::request& req) {
    auto json = crow::json::load(req.body);
    if (!json) {
      return crow::response(400);
    }
    auto data = json["data"];
    std::cout << "The user id is " << data << "\n";

    crow::json::wvalue x;
    x["data"] = data;
    return crow::response(x);
  });

  CROW_ROUTE(app, "/image")
      .methods("POST"_method)([&](const crow::request& req) {
        // Get user id and image from the request
        std::string id = req.headers.find(kUserIDHeaderValue)->second;
        std::string image = req.body;
        // Add a user to the environment or update an existing user
        auto points = environment_.updateClient(id, std::move(image));

        // Format points for transport - json
        std::vector<crow::json::wvalue> pointList;
        for (auto& point : points) {
          crow::json::wvalue newPoint;
          newPoint["x"] = point["x"];
          newPoint["y"] = point["y"];
          pointList.push_back(std::move(newPoint));
        }
        // Format JSON response
        crow::json::wvalue response;
        response["points"] = std::move(pointList);
        return crow::response(response);
      });

  // CROW_ROUTE(app, "/object")
  //     .methods("POST"_method)([&](const crow::request& req) {
  //       std::string id = req.headers.find(kUserIDHeaderValue)->second;

  //       // environment_.addObject();

  //     });
}
