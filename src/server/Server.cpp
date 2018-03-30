#include "include/server/Server.hpp"
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include "include/cv/SIFT.hpp"
#include "include/deps/crow.h"

const std::string kUserIDHeaderValue = "x-user-id";
const std::string kBestCandidateLocations = "x-points";

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
        // Get candidate points that ARKit detects
        std::string rawCandidatePoints;
        auto candidatePointHeader = req.headers.find(kBestCandidateLocations);
        if (candidatePointHeader != req.headers.end()) {
          rawCandidatePoints = candidatePointHeader->second;
        }
        /***** Process Candidate Points *****/
        // Collection of candidate points
        std::vector<cv::Point2f> candidatePoints;
        if (!rawCandidatePoints.empty()) {
          // Point strings
          std::vector<std::string> pointStrings;
          // Delimited by semicolon, i.e. 1,2;4,5;3,4
          boost::split(pointStrings, rawCandidatePoints, boost::is_any_of(";"));
          // Split each point
          for (auto pointString : pointStrings) {
            // Individual point string
            std::vector<std::string> aPointString;
            boost::split(aPointString, pointString, boost::is_any_of(","));
            candidatePoints.push_back(cv::Point2f(
                std::stof(aPointString[0]), std::stof(aPointString[1])));
          }
        }

        std::string image = req.body;
        // Add a user to the environment or update an existing user
        auto siftOutPoints =
            environment_.updateClient(id, std::move(image), candidatePoints);

        // Format points for transport - json
        std::vector<crow::json::wvalue> pointList;
        for (auto& aSiftPoint : siftOutPoints) {
          crow::json::wvalue newPoint;
          newPoint["x"] = aSiftPoint["x"];
          newPoint["y"] = aSiftPoint["y"];
          pointList.push_back(std::move(newPoint));
        }
        // Format JSON response
        crow::json::wvalue response;
        response["points"] = std::move(pointList);
        return crow::response(response);
      });

  CROW_ROUTE(app, "/object")
      .methods("POST"_method)([&](const crow::request& req) {
        std::string id = req.headers.find(kUserIDHeaderValue)->second;

        // environment_.addObject();
        crow::json::wvalue response;
        return crow::response(response);
      });

  CROW_ROUTE(app, "/sync")
      .methods("POST"_method)([&](const crow::request& req) {

        crow::json::wvalue response;
        response["objects"];
        return crow::response(response);
      });
}
