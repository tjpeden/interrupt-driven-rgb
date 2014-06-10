#include <SD.h>
#include <EEPROM.h>
#include <JsonParser.h>

#include <Arduino.h>

#include "waypoint_manager.h"

#define arraySize(a) (sizeof(a) / sizeof(a[0]))

WaypointManager::WaypointManager(uint8_t address) {
  waypoints = new LinkedList<Location>();
  index_address = address;
  index = readStoredIndex();
  if(index == 255) reset(); // EEPROM defaults to 255
}

bool
WaypointManager::loadWaypoints(const char* name) {
  File file;
  String raw;

  if((file = SD.open(name)) == NULL) return false;
  raw = file.readString();
  file.close();

  char json[raw.length()];
  raw.toCharArray(json, raw.length());

  JsonParser<32> parser;
  JsonHashTable hashTable = parser.parseHashTable(json);

  if(!hashTable.success()) return false;

  JsonArray waypoints = hashTable.getArray("waypoints");
  for(int i = 0; i < waypoints.getLength(); i++) {
    JsonArray point = waypoints.getArray(i);

    if(point.getLength() != 2) return false;

    Location l;
    l.latitude  = point.getDouble(0);
    l.longitude = point.getDouble(1);
    this->waypoints->add(l);
  }

  return true;
}

bool
WaypointManager::loadWaypoints(Location points[], size_t length) {
  for(int i = 0; i < length; i++) {
    this->waypoints->add(points[i]);
  }

  return true;
}

Location WaypointManager::current() {
  return waypoints->get(index);
}

void
WaypointManager::next() {
  setWaypoint(index + 1);
}

void
WaypointManager::reset() {
  setWaypoint(0);
}

bool
WaypointManager::hasNext() {
  return index <= waypoints->size();
}

char*
WaypointManager::toString() {
  char buffer[5];
  snprintf(buffer, sizeof(buffer), "%d/%d", index, waypoints->size());
  return buffer;
}

void
WaypointManager::setWaypoint(uint8_t i) {
  index = i;
  EEPROM.write(index_address, index);
}

uint8_t
WaypointManager::readStoredIndex() {
  return EEPROM.read(index_address);
}
