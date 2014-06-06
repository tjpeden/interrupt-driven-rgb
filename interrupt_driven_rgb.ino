// #include <avr/sleep.h>
// #include <pitches.h>

#include <SPI.h>
#include <Time.h>
// #include <EEPROM.h>
#include <Ucglib.h>
#include <TinyGPS++.h>
#include <Streaming.h>
#include <TimedEvent.h>
#include <ButtonEvent.h>
#include <FiniteStateMachine.h>

#define OLED_MOSI 11
#define OLED_CLK 13
#define OLED_CS 10
#define OLED_DC 3
#define OLED_RESET 2

#define GET_FIX_TIMER 1
#define READ_LOCATION_TIMER 2

#define LAT_INDEX 0
#define LNG_INDEX 1

#define arraySize(a) (sizeof(a) / sizeof(a[0]))

const uint8_t buttonPin = 2;
// const uint8_t id_address = 0;
const double threshold = 0.01;
// unsigned long start = 0;
double distanceTo;
double course;
uint8_t waypoint_index = 0;

const double waypoint[][2] = {
  { 36.139670, -96.004777 },
  { 36.139777, -96.003538 },
  { 36.140135, -96.004070 },
};

// U8GLIB_SSD1351_128X128_HICOLOR oled(OLED_CS, OLED_DC, OLED_RESET);
Ucglib_SSD1351_18x128x128_HWSPI oled(OLED_DC, OLED_CS, OLED_RESET);

TinyGPSPlus gps;

// TinyGPSCustom ack_command(gps, "PMTK001", 1);
// TinyGPSCustom ack_response(gps, "PMTK001", 2);
// TinyGPSCustom awake(gps, "PMTK010", 1);

State GetFix       = State(enterGetFix, updateGetFix, exitGetFix);
State ReadLocation = State(enterReadLocation, updateReadLocation, exitReadLocation);
State SleepModules = State(enterSleepModules, updateSleepModules, NULL);
State SleepNow     = State(updateSleepNow);
State WakeModules  = State(enterWakeModules, updateWakeModules, NULL);
State NextWaypoint = State(enterNextWaypoint, updateNextWaypoint, NULL);
State LastWaypoint = State(updateLastWaypoint);
State GPSFailure   = State(enterGPSFailure, NULL, NULL);

FSM stateMachine = FSM(GetFix);

// void pictureLoop(void (draw)()) {
//   oled.firstPage();
//   do {
//     draw();
//   } while(oled.nextPage());
// }

void onDown(ButtonInformation* sender) { /* do nothing here */ }

void onUp(ButtonInformation* sender) { /* do nothing here */ }

void onDouble(ButtonInformation* sender) { /* do nothing here */ }

void onHold(ButtonInformation* sender) {
  if(millis() < 6000) {
    // reset waypoint index
  } else {
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
  oled.setFont(ucg_font_fixed_v0r);

  oled.setColor(255, 255, 0);
  oled.setPrintPos(0, oled.getHeight() / 2);
  oled << F("Please wait...");

  int width = oled.getStrWidth("0 satellites");
  oled.setClipRange(0, oled.getHeight() - 12, width, 12);

  TimedEvent.start(GET_FIX_TIMER);
}

void updateGetFix() {
  if(gps.location.isValid())
    stateMachine.transitionTo(ReadLocation);

  oled.setColor(255, 255, 255);

  oled.setPrintPos(0, oled.getHeight() + oled.getFontDescent());
  oled << gps.satellites.value() << F(" satellites");
}

void exitGetFix() {
  TimedEvent.stop(GET_FIX_TIMER);
}

void enterReadLocation() {
  drawTitle(F("Directions"));
  oled.setClipRange(0, 22, oled.getWidth(), oled.getHeight() - 22);

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

  if((gps.date.isUpdated() && gps.date.age() < 200) || (gps.time.isUpdated() && gps.time.age() < 200)) {
    setTime(
      gps.time.hour(),
      gps.time.minute(),
      gps.time.second(),
      gps.date.day(),
      gps.date.month(),
      gps.date.year()
    );
    adjustTime(-6 * SECS_PER_HOUR);
  }

  drawReadLocation();
}

void exitReadLocation() {
  TimedEvent.stop(READ_LOCATION_TIMER);
}

void enterSleepModules() {
  drawTitle(F("Going to Sleep"));
  oled.setFont(ucg_font_fixed_v0r);

  oled.setColor(255, 255, 0);
  oled.setPrintPos(0, oled.getHeight() / 2);
  oled << F("Please wait...");
}

void updateSleepModules() {
}

void updateSleepNow() {
}

void enterWakeModules() {
  ;
}

void updateWakeModules() {
  drawWakeModules();
}

void enterNextWaypoint() {
  ;
}

void updateNextWaypoint() {
  drawNextWaypoint();
}

void updateLastWaypoint() {
  drawLastWaypoint();
}

void enterGPSFailure() {
  drawGPSFailure();
}

void drawReadLocation() {
  char datetime[24];
  char status[24];

  sprintf(datetime, "%d-%02d-%02d    %02d:%02d:%02d", year(), month(), day(), hourFormat12(), minute(), second());
  sprintf(status, "%2d satellites", gps.satellites.value());

  oled.setFont(ucg_font_7x13Br);

  oled.setColor(0, 0, 255);
  oled.drawRFrame(0, 0, oled.getWidth(), 20, 3);


  oled.setColor(255, 255, 255);
  oled.setPrintPos(5, 15);
  oled << F("Directions");

  oled.setFont(ucg_font_fixed_v0r);

  oled.setColor(255, 255, 255);
  oled.setPrintPos(0, 22);
  oled << datetime;

  oled.setFont(ucg_font_7x13Br);

  oled.setPrintPos(0, oled.getHeight() / 2 - 20);
  oled << distanceTo << F(" miles");

  oled.setPrintPos(0, oled.getHeight() / 2 + 20);
  oled << TinyGPSPlus::cardinal(course);

  oled.setFont(ucg_font_fixed_v0r);

  oled.setPrintPos(0, oled.getHeight() - (oled.getFontDescent() + 12));
  oled << status;

  oled.setPrintPos(0, oled.getHeight() - (oled.getFontDescent() + 24));
  oled << F("hdop: ") << gps.hdop.value() << F(" WP: ") << (waypoint_index + 1) << '/' << arraySize(waypoint);
}

void drawSleepModules() {
  oled.clearScreen();
  oled.setColor(0, 0, 255);
  oled.drawRFrame(0, 0, oled.getWidth(), 20, 3);

  oled.setFont(ucg_font_7x13Br);

  oled.setColor(255, 255, 255);
  oled.setPrintPos(5, 15);
  oled << F("Sleep Modules");
}

void drawSleepNow() {
  ;
}

void drawWakeModules() {
  ;
}

void drawNextWaypoint() {
  oled.clearScreen();
  oled.setColor(0, 0, 255);
  oled.drawRFrame(0, 0, oled.getWidth(), 20, 3);

  oled.setFont(ucg_font_7x13Br);

  oled.setColor(255, 255, 255);
  oled.setPrintPos(5, 15);
  oled << F("Next Waypoint");
}

void drawLastWaypoint() {
  ;
}

void drawGPSFailure() {
  oled.clearScreen();
  oled.setColor(0, 0, 255);
  oled.drawRFrame(0, 0, oled.getWidth(), 20, 3);

  oled.setFont(ucg_font_7x13Br);

  oled.setColor(255, 255, 255);
  oled.setPrintPos(5, 15);
  oled << F("GPS Failure");

  oled.setFont(ucg_font_fixed_v0r);

  oled.setColor(255, 255, 0);
  oled.setPrintPos(0, oled.getHeight() / 2);
  oled << F("No data from GPS!");
}

void setup() {
  ButtonEvent.addButton(
    buttonPin,
    onDown,
    onUp,
    onHold,
    5000,
    onDouble,
    200
  );

  TimedEvent.addTimer(GET_FIX_TIMER, 5000, gpsFailure);
  TimedEvent.addTimer(READ_LOCATION_TIMER, 60000, initiateSleep);

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
  ButtonEvent.loop();
  stateMachine.update();
}
