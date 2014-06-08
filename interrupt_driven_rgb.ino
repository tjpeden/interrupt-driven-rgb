// #include <pitches.h>

#include <SPI.h>
#include <Time.h>
// #include <EEPROM.h>
#include <Button.h>
#include <Ucglib.h>
#include <TinyGPS++.h>
#include <Streaming.h>
#include <TimedEvent.h>
#include <LowPower_Teensy3.h>
#include <FiniteStateMachine.h>

// SPI Pins
#define OLED_MOSI 11
#define OLED_CLK 13
#define OLED_CS 10
#define OLED_DC 3
#define OLED_RESET 2

// Timer IDs
#define GET_FIX_TIMER 1
#define READ_LOCATION_TIMER 2

// Index Labels
#define LAT_INDEX 0
#define LNG_INDEX 1

#define arraySize(a) (sizeof(a) / sizeof(a[0]))

const uint8_t buttonPin = 6;
// const uint8_t id_address = 0;
const double threshold = 0.01;
double distanceTo;
double course;
uint8_t waypoint_index = 0;

const double waypoint[][2] = {
  { 36.049521, -95.921004 },
  { 36.048774, -95.921697 },
  { 36.048706, -95.920835 },
};

Button button = Button(buttonPin);

Ucglib_SSD1351_18x128x128_HWSPI oled(OLED_DC, OLED_CS, OLED_RESET);

TEENSY3_LP LP = TEENSY3_LP();

TinyGPSPlus gps;

// Custom NMEA parsers
TinyGPSCustom ack_command(gps, "PMTK001", 1);
TinyGPSCustom ack_response(gps, "PMTK001", 2);
TinyGPSCustom awake(gps, "PMTK010", 1);

// Internal States
State GetFix       = State(enterGetFix, updateGetFix, exitGetFix);
State ReadLocation = State(enterReadLocation, updateReadLocation, exitReadLocation);
State SleepModules = State(enterSleepModules, updateSleepModules, NULL);
State SleepNow     = State(updateSleepNow);
State WakeModules  = State(enterWakeModules, updateWakeModules, NULL);
State NextWaypoint = State(enterNextWaypoint, NULL, NULL);
State LastWaypoint = State(enterLastWaypoint, NULL, NULL);
State GPSFailure   = State(enterGPSFailure, NULL, NULL);

FSM stateMachine = FSM(GetFix);

// Event handlers
void onClick(Button& button) {
  if(stateMachine.isInState(NextWaypoint)) {
    if(waypoint_index >= arraySize(waypoint)) {
      stateMachine.transitionTo(LastWaypoint);
    } else {
      stateMachine.transitionTo(ReadLocation);
    }
  }
}

void onHold(Button& button) {
  if(millis() < 6000 || stateMachine.isInState(LastWaypoint)) {
    waypoint_index = 0;
  } else {
    if(gps.location.isValid())
      stateMachine.transitionTo(NextWaypoint);
  }
}

void gpsFailure(TimerInformation* sender) {
  if(gps.charsProcessed() < 10)
    stateMachine.transitionTo(GPSFailure);
}

void initiateSleep(TimerInformation* sender) {
  stateMachine.transitionTo(SleepModules);
}

void drawTitle(const __FlashStringHelper *ifsh) {
  oled.undoClipRange();
  oled.clearScreen();

  oled.setColor(0, 0, 255);
  oled.drawRFrame(0, 0, oled.getWidth(), 20, 3);

  oled.setFont(ucg_font_7x13Br);
  oled.setColor(255, 255, 255);
  oled.setPrintPos(5, 14);
  oled << ifsh;
}

// States

void enterGetFix() {
  drawTitle(F("Aquiring Fix"));
  oled.setFont(ucg_font_6x12r);

  oled.setColor(255, 255, 0);
  oled.setPrintPos(0, oled.getHeight() / 2);
  oled << F("Please wait...");

  delay(1000);
  TimedEvent.start(GET_FIX_TIMER);
}

void updateGetFix() {
  if(gps.location.isValid() && gps.location.isUpdated() && gps.location.age() < 200)
    stateMachine.transitionTo(ReadLocation);

  oled.setColor(255, 255, 255);

  oled.setPrintPos(0, oled.getHeight() + oled.getFontDescent());
  oled.printf(F("%d satellites  "), gps.satellites.value());
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
    distanceTo =
      TinyGPSPlus::distanceBetween(
        gps.location.lat(),
        gps.location.lng(),
        waypoint[waypoint_index][LAT_INDEX],
        waypoint[waypoint_index][LNG_INDEX]
      ) * _GPS_MILES_PER_METER;
    course =
      TinyGPSPlus::courseTo(
        gps.location.lat(),
        gps.location.lng(),
        waypoint[waypoint_index][LAT_INDEX],
        waypoint[waypoint_index][LNG_INDEX]
      );

    if(distanceTo < threshold)
      stateMachine.transitionTo(NextWaypoint);
  }

  drawReadLocation();
}

void exitReadLocation() {
  TimedEvent.stop(READ_LOCATION_TIMER);
}

void enterSleepModules() {
  drawTitle(F("Going to Sleep"));
  oled.setFont(ucg_font_6x12r);

  oled.setColor(255, 255, 0);
  oled.setPrintPos(0, oled.getHeight() / 2);
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
  oled.setPrintPos(0, oled.getHeight() / 2);
  oled << F("Please wait...");

  Serial3.println(' ');
}

void updateWakeModules() {
  if(awake.isUpdated()) {
    awake.value();
    stateMachine.transitionTo(GetFix);
  }
}

void enterNextWaypoint() {
  drawTitle(F("Waypoint Reached"));
  oled.setFont(ucg_font_6x12r);

  oled.setColor(255, 255, 0);
  oled.setPrintPos(0, oled.getHeight() / 2);
  oled << F("Push the button to");
  oled.setPrintPos(0, (oled.getHeight() / 2) + (oled.getFontAscent() - oled.getFontDescent()));
  oled << F("continue.");

  waypoint_index++;
}

void enterLastWaypoint() {
  drawTitle(F("Open the Box"));
}

void enterGPSFailure() {
  drawTitle(F("GPS Failure"));
  oled.setFont(ucg_font_6x12r);

  oled.setColor(255, 255, 0);
  oled.setPrintPos(0, oled.getHeight() / 2);
  oled << F("No data from GPS!");
}

void drawReadLocation() {
  // Distance as a formated string
  char _distanceTo[14];
  sprintf(_distanceTo, "%.2f mi\0", distanceTo);

  // Debug info formatted
  char _debug[21];
  sprintf(_debug, "hdop: %d WP: %d/%d", gps.hdop.value(), (waypoint_index + 1), arraySize(waypoint));

  oled.setFont(ucg_font_9x15Br);
  oled.setColor(0, 255, 0);

  oled.setPrintPos(0, oled.getHeight() / 2);
  oled.printf(F("%-14s"), _distanceTo);

  oled.setPrintPos(0, (oled.getHeight() / 2) + (oled.getFontAscent() - oled.getFontDescent()));
  oled.printf(F("%-14s"), TinyGPSPlus::cardinal(course));

  oled.setFont(ucg_font_6x12r);
  oled.setColor(255, 255, 255);

  oled.setPrintPos(0, oled.getHeight() + oled.getFontDescent());
  oled.printf(F("%d satellites  "), gps.satellites.value());

  // Debug info
  oled.setPrintPos(0, (oled.getHeight() + oled.getFontDescent()) - 12);
  oled.printf(F("%-21s"), _debug);
}

void setup() {
  button.clickHandler(onClick);
  button.holdHandler(onHold, 5000);

  TimedEvent.addTimer(GET_FIX_TIMER, 5000, gpsFailure);
  TimedEvent.addTimer(READ_LOCATION_TIMER, 60000*5, initiateSleep);

  Serial3.begin(9600);
  delay(1000);
  Serial3.println(F("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"));
  Serial3.println(F("$PMTK220,200*2C"));

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
