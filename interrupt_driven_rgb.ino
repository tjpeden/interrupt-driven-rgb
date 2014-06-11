// #include <pitches.h>

#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Button.h>
#include <Ucglib.h>
#include <TinyGPS++.h>
#include <Streaming.h>
#include <JsonParser.h>
#include <LinkedList.h>
#include <TimedEvent.h>
#include <LowPower_Teensy3.h>
#include <FiniteStateMachine.h>

#include "interrupt_driven_rgb.h"
#include "waypoint_manager.h"

// SPI Pins
#define OLED_MOSI 11
#define OLED_CLK 13
#define OLED_CS 14
#define OLED_DC 3
#define OLED_RESET 2
#define SD_MISO 12
#define SD_CS 10

// Timer IDs
#define GET_FIX_TIMER 1
#define READ_LOCATION_TIMER 2

// Index Labels
#define LAT_INDEX 0
#define LNG_INDEX 1

#define arraySize(a) (sizeof(a) / sizeof(a[0]))

const uint8_t buttonPin = 6;
const double threshold = 0.0075; // ~40 ft seems optimal
double distanceTo;
double course;

WaypointManager waypoint_manager(0);

Button button = Button(buttonPin, BUTTON_PULLUP_INTERNAL, true, 100);

Ucglib_SSD1351_18x128x128_HWSPI oled(OLED_DC, OLED_CS, OLED_RESET);

TEENSY3_LP LP = TEENSY3_LP();

TinyGPSPlus gps;

// Custom NMEA parsers
TinyGPSCustom ack_command(gps, "PMTK001", 1);
TinyGPSCustom ack_response(gps, "PMTK001", 2);
TinyGPSCustom awake(gps, "PMTK010", 1);

// Internal States
State Loading      = State(enterLoading, updateLoading, NULL);
State GetFix       = State(enterGetFix, updateGetFix, exitGetFix);
State ReadLocation = State(enterReadLocation, updateReadLocation, exitReadLocation);
State SleepModules = State(enterSleepModules, updateSleepModules, NULL);
State SleepNow     = State(updateSleepNow);
State WakeModules  = State(enterWakeModules, updateWakeModules, NULL);
State Checkpoint   = State(enterCheckpoint, NULL, NULL);
State OpenBox      = State(enterOpenBox, NULL, NULL);
State GPSFailure   = State(enterGPSFailure, NULL, NULL);
State SDFailure    = State(enterSDFailure, NULL, NULL);

FSM stateMachine = FSM(Loading);

// Event handlers
void onPress(Button& button) {
  if(stateMachine.isInState(Checkpoint)) {
    if(waypoint_manager.hasNext()) {
      stateMachine.transitionTo(ReadLocation);
    } else {
      stateMachine.transitionTo(OpenBox);
    }
  }
}

void onHold(Button& button) {
  if(millis() < 20000 || stateMachine.isInState(OpenBox)) {
    waypoint_manager.reset();
  } else {
    if(gps.location.isValid()) stateMachine.transitionTo(Checkpoint);
  }
}

void gpsFailure(TimerInformation* sender) {
  if(gps.charsProcessed() < 10)
    stateMachine.transitionTo(GPSFailure);
}

void initiateSleep(TimerInformation* sender) {
  stateMachine.transitionTo(SleepModules);
}

void drawTitleBorder() {
  oled.clearScreen();

  oled.setColor(0, 0, 255);
  oled.drawRFrame(0, 0, oled.getWidth(), 20, 3);

  oled.setFont(ucg_font_7x13Br);
  oled.setColor(255, 255, 255);
  oled.setPrintPos(5, 14);
}

void drawTitle(const __FlashStringHelper *ifsh) {
  drawTitleBorder();
  oled << ifsh;
}

void drawTitle(char *text) {
  drawTitleBorder();
  oled << text;
}

uint8_t l(uint8_t offset, Base base) {
  uint8_t baseline = 0;
  switch(base) {
    case TOP:
    baseline = 24 + oled.getFontAscent();
    break;
    case CENTER:
    baseline = oled.getHeight() / 2;
    break;
    case BOTTOM:
    baseline = oled.getHeight() + oled.getFontDescent();
    break;
  }

  baseline += (oled.getFontAscent() - oled.getFontDescent()) * offset;

  return baseline;
}

void describe(char* text, uint8_t width) {
  uint8_t index, line = 0;
  String string = text;

  for(uint8_t i = 0, j = width; j < string.length(); j = index + width, i = index + 1) {
    index = string.lastIndexOf(' ', j);
    if(index < 0) index = j;

    oled.setPrintPos(0, l(line, TOP));
    oled << string.substring(i, index);
    line++;
  }

  oled.setPrintPos(0, l(line, TOP));
  oled << string.substring(index+1);
}

// States
void enterLoading() {
  drawTitle(F("Loading"));
  oled.setFont(ucg_font_6x12r);

  oled.setColor(255, 255, 0);
  oled.setPrintPos(0, l(0, CENTER));
  oled << F("Please wait...");
}

void updateLoading() {
  if(!SD.begin(SD_CS)) {
    stateMachine.transitionTo(SDFailure);
    return;
  }

  if(!waypoint_manager.loadWaypoints("settings.txt")) {
    stateMachine.transitionTo(SDFailure);
    return;
  }
  // waypoint_manager.debug(&Serial);

  delay(1000); // prevent ugly flicker

  // Prevent the display from going crazy
  oled.begin(UCG_FONT_MODE_SOLID);
  oled.setColor(1, 0, 0, 0);

  if(waypoint_manager.hasNext()) {
    stateMachine.transitionTo(GetFix);
  } else {
    stateMachine.transitionTo(OpenBox);
  }
}

void enterGetFix() {
  drawTitle(F("Aquiring Fix"));
  oled.setFont(ucg_font_6x12r);

  oled.setColor(255, 255, 0);
  oled.setPrintPos(0, l(0, CENTER));
  oled << F("Please wait...");

  delay(1000);
  TimedEvent.start(GET_FIX_TIMER);
}

void updateGetFix() {
  if(gps.location.isValid() && gps.location.isUpdated() && gps.location.age() < 200)
    stateMachine.transitionTo(ReadLocation);

  if(gps.satellites.isUpdated()) {
    oled.setColor(255, 255, 255);

    oled.setPrintPos(0, l(0, BOTTOM));
    oled.printf(F("%d satellites  "), gps.satellites.value());
  }
}

void exitGetFix() {
  TimedEvent.stop(GET_FIX_TIMER);
}

void enterReadLocation() {
  drawTitle(F("Directions"));

  TimedEvent.start(READ_LOCATION_TIMER);
}

void updateReadLocation() {
  if(gps.location.isUpdated() && gps.location.age() < 200) {
    Location waypoint = waypoint_manager.current();

    distanceTo =
      TinyGPSPlus::distanceBetween(
        gps.location.lat(),
        gps.location.lng(),
        waypoint.latitude,
        waypoint.longitude
      ) * _GPS_MILES_PER_METER;
    course =
      TinyGPSPlus::courseTo(
        gps.location.lat(),
        gps.location.lng(),
        waypoint.latitude,
        waypoint.longitude
      );

    if(distanceTo < threshold)
      stateMachine.transitionTo(Checkpoint);

    drawReadLocation();
  }
}

void exitReadLocation() {
  TimedEvent.stop(READ_LOCATION_TIMER);
}

void enterSleepModules() {
  drawTitle(F("Going to Sleep"));
  oled.setFont(ucg_font_6x12r);

  oled.setColor(255, 255, 0);
  oled.setPrintPos(0, l(0, CENTER));
  oled << F("Please wait...");

  Serial3.println(F("$PMTK161,0*28")); // Sleep GPS
}


void updateSleepModules() {
  if(ack_command.isUpdated())
    if(strcmp(ack_command.value(), "161") + strcmp(ack_response.value(), "3") == 0)
      stateMachine.transitionTo(SleepNow);
}

void updateSleepNow() {
  oled.clearScreen();

  LP.DeepSleep(GPIO_WAKE, PIN_6);

  stateMachine.transitionTo(WakeModules);
}

void enterWakeModules() {
  drawTitle(F("Waking"));
  oled.setFont(ucg_font_6x12r);

  oled.setColor(255, 255, 0);
  oled.setPrintPos(0, l(0, CENTER));
  oled << F("Please wait...");

  Serial3.println(' ');
}

void updateWakeModules() {
  if(awake.isUpdated()) {
    awake.value();
    stateMachine.transitionTo(GetFix);
  }
}

void enterCheckpoint() {
  Location current = waypoint_manager.current();

  drawTitle(current.name);
  oled.setFont(ucg_font_6x12r);

  oled.setColor(0, 255, 0);
  describe(current.description, 21);

  oled.setColor(255, 255, 0);
  oled.setPrintPos(0, l(-1, BOTTOM));
  oled << F("Push the button to");
  oled.setPrintPos(0, l(0, BOTTOM));
  oled << F("continue.");

  waypoint_manager.next();
}

void enterOpenBox() {
  drawTitle(F("Open the Box"));
  oled.setFont(ucg_font_9x15Br);

  oled.setColor(0, 255, 0);
  oled.setPrintPos(0, l(0, CENTER));
  oled << F("You made it!");
  oled.setPrintPos(0, l(1, CENTER));
  oled << F("You may open");
  oled.setPrintPos(0, l(2, CENTER));
  oled << F("the box.");
}

void enterGPSFailure() {
  drawTitle(F("GPS Failure"));
  oled.setFont(ucg_font_6x12r);

  oled.setColor(255, 0, 0);
  oled.setPrintPos(0, l(0, CENTER));
  oled << F("No data from GPS!");
}

void enterSDFailure() {
  drawTitle(F("SD Card Failure"));
  oled.setFont(ucg_font_6x12r);

  oled.setColor(255, 0, 0);
  oled.setPrintPos(0, l(0, CENTER));
  oled << F("There was a problem");
  oled.setPrintPos(0, l(1, CENTER));
  oled << F("reading the SD card.");
}

void drawReadLocation() {
  // Distance as a formated string
  char _distanceTo[14];
  if(distanceTo < 0.1) {
    double ft = distanceTo * 5280;
    sprintf(_distanceTo, "%.0f ft", ft);
  } else {
    sprintf(_distanceTo, "%.2f mi", distanceTo);
  }

  // Debug info formatted
  char _stats[8];
  char _debug[21];
  waypoint_manager.stats(_stats);
  sprintf(_debug, "hdop: %d WP: %s", gps.hdop.value(), _stats);

  oled.setFont(ucg_font_9x15Br);
  oled.setColor(0, 255, 0);

  oled.setPrintPos(0, l(0, CENTER));
  oled.printf(F("%-14s"), _distanceTo);

  oled.setPrintPos(0, l(1, CENTER));
  oled.printf(F("%-14s"), TinyGPSPlus::cardinal(course));

  oled.setFont(ucg_font_6x12r);
  oled.setColor(255, 255, 255);

  oled.setPrintPos(0, l(0, BOTTOM));
  oled.printf(F("%d satellites  "), gps.satellites.value());

  // Debug info
  oled.setPrintPos(0, l(-1, BOTTOM));
  oled.printf(F("%-21s"), _debug);
}

void setup() {
  pinMode(SD_CS, OUTPUT);

  button.pressHandler(onPress);
  button.holdHandler(onHold, 15000);

  TimedEvent.addTimer(GET_FIX_TIMER, 5000, gpsFailure);
  TimedEvent.addTimer(READ_LOCATION_TIMER, 60000*5, initiateSleep);

  Serial3.begin(9600);
  delay(1000);
  Serial3.println(F("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"));
  Serial3.println(F("$PMTK220,1000*1F"));

  oled.begin(UCG_FONT_MODE_SOLID);
  oled.setColor(1, 0, 0, 0);
  oled.clearScreen();
}

void loop() {
  while(Serial3.available()) gps << Serial3.read();
  TimedEvent.loop();
  button.process();
  stateMachine.update();
}
