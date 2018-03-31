#include "include/server/Server.hpp"
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include "include/cv/SIFT.hpp"
#include "include/deps/crow.h"

const std::string kUserIDHeaderValue = "x-user-id";
const std::string kBestCandidateLocations = "x-points";
const std::string kObjectUpdateIDHeader = "x-object-id";
const std::string kObjectUpdateLocationValue = "x-object-location";

Server::Server() {}

void Server::run(uint32_t port) {
  // Start main server thread
  app.port(port).multithreaded().run();
}

crow::json::wvalue Server::mapToCrowWValue(
    std::unordered_map<std::string, std::string> map) {
  crow::json::wvalue val;
  for (auto& kv : map) {
    val[kv.first] = kv.second;
  }
  return val;
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

  /**
   * Processes an orientation image from a client. Runs SIFT on the incoming
   * image in conjunction with an image from another client, and computes a
   * homography and perspective transform while finding the local candidate
   * coordinates from ARKit that are a best fit (as for these points, there is
   * better information about their 2D-3D mapping).
   *
   * @header user id: the id of the user sending the image.
   * @header candidate points: a semicolon-delimited list of comma-delimited 2D
   *     points that are good candidates as per ARKit
   * @response: four points computed via a homography and perspective transform
   *     which are candidate points closest to the points computed by ARKit.
   */
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
        // Parse request string into list of L2-calcuable points
        // Final collection of candidate points
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
          pointList.push_back(mapToCrowWValue(aSiftPoint));
        }
        // Format JSON response
        crow::json::wvalue response;
        response["points"] = std::move(pointList);
        return crow::response(response);
      });

  /**
   * Endpoint for object manipulation in the environment
   *
   * @header object id: the id of the object being manipulated. Empty if a new
   * object is being created.
   * @header location: the location of the new object or of the new position of
   * the object being updated.
   * @response: the id of the new or modified object.
   */
  CROW_ROUTE(app, "/object")
      .methods("POST"_method)([&](const crow::request& req) {
        // Get the updated location of the object in question
        std::string rawLocation =
            req.headers.find(kObjectUpdateLocationValue)->second;
        // Parse location into coordinates
        // TODO: this
        Location objectLocation;
        // See if we're working with an existing object ID. If so, go forth and
        // update it: otherwise, create a new object. Always respond with the ID
        std::string objectID;
        auto objectIDHeader = req.headers.find(kObjectUpdateIDHeader);
        if (objectIDHeader != req.headers.end()) {
          // Modify existing object
          objectID = objectIDHeader->second;
          environment_.updateObject(objectID, objectLocation);
        } else {
          // Create a new object with a sequentially-generated ID
          objectID = environment_.addObject();
        }
        // Format JSON response
        crow::json::wvalue response;
        response["objectID"] = std::move(objectID);
        return crow::response(response);
      });

  /**
   * Returns a description of the augmented reality environment. This includes:
   * - All AR objects as created by clients.
   *
   * @response: json payload with data
   *
   * Serialization format (json):
   * {
   *    objects: [
   *      {
   *        id: [id],
   *        x: [double],
   *        y: [double],
   *        z: [double]
   *      }
   *    ],
   *    ...
   * }
   */
  CROW_ROUTE(app, "/sync")
      .methods("POST"_method)([&](const crow::request& req) {
        // Get environment data
        auto objectData = environment_.getObjectRepresentation();

        // Serialize
        std::vector<crow::json::wvalue> objectList;
        for (auto& object : objectData) {
          objectList.push_back(mapToCrowWValue(object));
        }
        // Format JSON response
        crow::json::wvalue response;
        response["objects"] = std::move(objectList);
        return crow::response(response);
      });
}
