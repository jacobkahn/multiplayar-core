#include "include/environment/Environment.hpp"
#include <boost/functional/hash.hpp>
#include <memory>
#include <utility>
#include <vector>
#include "include/environment/Client.hpp"
#include "include/environment/Object.hpp"

PointList Environment::updateClient(EntityID id, std::string image) {
  std::shared_ptr<Client> client;
  if (clientExists(id)) {
    client = clientsByID.find(id)->second;
    client->processImage(std::move(image));

  } else {
    client = std::make_shared<Client>(id);
    client->processImage(std::move(image));
    clients.push_back(client);
    clientsByID.insert(std::make_pair(id, client));
  }

  // Attempt to compute homography with another client
  // Make sure we're not the only client
  if (clients.size() > 1) {
    // Get not you
    for (auto& otherClient : clients) {
      if (otherClient->getID() != client->getID()) {
        // We found another client that isn't us. Compute homography
        auto siftClient = std::make_unique<SIFTClient>();
        auto result = siftClient->computeHomographyTransformationFromClients(
            client, otherClient);
        // We only care about the first point list - that's the calling user's
        return result.first;
      }
    }
  }
  return {};
}

bool Environment::clientExists(const EntityID& id) {
  return clientsByID.find(id) != clientsByID.end();
}

void Environment::addObject(EntityID id) {
  auto object = std::make_shared<Object>(id);

  objects.push_back(object);
  objectsByID.insert(std::make_pair(id, object));
}