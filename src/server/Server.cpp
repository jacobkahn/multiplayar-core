#include "include/server/Server.hpp"
#include <sstream>
#include <string>
#include <thread>
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

  CROW_ROUTE(app, "/post")
      .methods("POST"_method)([](const crow::request& req) {
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
}
