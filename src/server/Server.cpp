#include "include/server/Server.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include "include/cv/SIFT.hpp"
#include "include/deps/crow.h"

Server::Server() {}

void Server::run(uint32_t port) {
  // Start main server thread
  app.port(port).multithreaded().run();
}

void Server::setup() {
  // Sample routing
  CROW_ROUTE(app, "/get")
  ([] {
    crow::json::wvalue x;
    x["data"] = "data";
    return x;
  });

  CROW_ROUTE(app, "/post").methods("POST"_method)([](const crow::request& req) {
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
        std::string id =
            req.headers.find("x-user-id")->second; // TODO: get the id
        std::string image = req.body; // TODO: get the image
        // Add a user to the environment or update an existing user
        auto points = environment_->updateClient(id, image);

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
}
