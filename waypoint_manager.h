#ifndef WAYPOINT_MANAGER_H_
#define WAYPOINT_MANAGER_H_

#include <LinkedList.h>

typedef struct Location {
  double latitude;
  double longitude;
} Location;

class WaypointManager {
public:
  WaypointManager(uint8_t);
  bool loadWaypoints(const char*);
  bool loadWaypoints(Location[], size_t);
  Location current();
  void next();
  void reset();
  bool hasNext();
  char* toString();

private:
  LinkedList<Location> *waypoints;
  uint8_t index_address;
  uint8_t index;

  void setWaypoint(uint8_t);
  uint8_t readStoredIndex();
};

#endif /* WAYPOINT_MANAGER_H_ */
