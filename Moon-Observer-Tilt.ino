/*
This sketch displays the moon phases using 40 images (9 degree increwments) but will show the current phase realistic with the tilted illuminated limb progressing over time.
For this it takes the observers position on earth into account. The formulae used are based on Jean Meeus collection of "Astronomical Algorithms" from 1998.
I also wanted to calculated the next upcoming black and blue moons and the current azimuth of sun and moon as well as the relation to each other.
This requires a more precise calculation than the commonly used average of 29.53 days per moon cycle. For calculating a seasonal blue moon, we also need to 
calculate solstices and equinoxes.
Calculating the angle of the illuminated limb based on Meeus' formula results in the angle starting on top of the moon increasing CCW. For easier readability, 
I inverted the angle to increase CW.
While the moon is below the horizon (Alt < 0) the display is hypothetical and purposefully ignoring that the Earth is getting in the line of sight.

The sketch is written for an ESP32-Wroom, but any other ESP32 dev board etc should work.
I did not get the SD-Card holding the PNG images to work on the same SPI as the two ILI9314 displays, so I'm using two SPI busses. 
The SD card is plugged into one of the slots on one of the two displays.
A rotary encoder is used to adjust Time/Date/Timezone/Observer-Lat/Lon.
Long-press to enter settings, showing a box at the selected field. Changing values by turning, short click advances to next setting.
For setting Lat/Long get the current location from google searching for e.g. "lat lon minneapolis" and round the coordinates to the next full degree.
No need for higher precision. Lat/Lon coordinates are usually expressed using N/S/W/E. For S and W use negative, for N and E use positive values
E.g. US/Minneapolis 44.9778° N, 93.2650° W, adjust settings to Lat 45 and Lon -93.

The selected timezone and lat/lon coordinates are stored in RTC NVRAM. I added a couple of Timezones (USA/Brittain/Western Europe). Modify the array below as needed.
The RTC is running on UTC as all calculations use UTC.
I removed the battery holder from the RTC module because due to a design flaw the LIR2032 will be overcharged continuously. I replaced it by a 1.5F supercap 

All PIN connections and their usage are listed in the remaks below.

*/

#include <TFT_eSPI.h>            // TFT library, set display type in  file "User_Setup.h" in Libray main or add to local directory
#include <SPI.h>                 // SPI bus for TFT and SD-Card
#include <SD.h>                  // SD card file system
#include <PNGdec.h>              // Read PNG files from SD card. White pixels are treated as transparent (0xFFFF)
#include <Wire.h>                // I2C Library for RTC
#include <RTClib.h>              // RTC (replaced LIR2032 by a 1.5F 5.5V SuperCap, due to bad charging design, LIR being constantly overcharged and died within a year)
#include <ESP32RotaryEncoder.h>  // Rotary and button handling
#include <TimeLib.h>             // for time calculations / timezone
#define FONT1 &FreeMono9pt7b     // Simple Font included with TFT_eSPI library


/*
   Define SPI PINs (may need running Display and SD card on seaparate SPI busses)
   Display TFT_eSPI Pins defined in User_Setup.h
   TFT_MISO 19  // unused in this sketch
   TFT_MOSI 23
   TFT_SCLK 18
   TFT_CS   35  // Chip select control pin using an input pin, effectively removing control from library for CS
   TFT_DC    2  // Data Command control pin
   TFT_RST   4  // Reset pin (could connect to RST pin)
*/
#define DispText_CS 5   // manual CS switch outside Library, defined CS Pin in library as 35 to take manual control (User_Setup.h)
#define DispMoon_CS 16  // display for moon phase and relative position of Sun,Moon,Earth

// SD Card SPI connections
#define MOSI 27
#define MISO 26
#define SCLK 14
#define SD_CS 15

// I2C Bus Pins
#define I2C_SDA 21  // default pins
#define I2C_SCL 22

// Encoder Pins
#define RotA 33                           // Rotary Pin A (swap these pins if rotary sense is inverted, or use knob->invertDirection(); in setup)
#define RotB 32                           // Rotary Pin B
#define BtnPin 25                         // Encoder Push Button
RotaryEncoder rotaryEncoder(RotA, RotB);  // Library supports button press as well, but causes Core 0 panic on an ESP-C3 due to interrupt handler timeout
bool SelBtnOld = false;                   // remember previous button status
unsigned long BtnRelease = 0;             // button release timestamp

TFT_eSPI tft = TFT_eSPI();
unsigned long displayRefresh = 500;    // reduced refresh time in ms during settings screen to avoid extensive flicker
unsigned long displayRefreshNext = 0;  // next scheduled refresh time stamp (millis)
#define TFT_COLOR1 0xCD0C              // old (warm) incandescent color, adjust as desired
uint16_t TFT_BOXES = TFT_COLOR1;       // settings boxes
PNG png;                               // PNG image handling

// Image dimensions for PNG files being rotated (Moon illuminated disk)
#define MAX_IMAGE_WIDTH 164
#define MAX_IMAGE_HEIGHT 164

// Use row pointers to create the image buffer
uint16_t *originalImage[MAX_IMAGE_HEIGHT];
uint16_t *rotatedImage[MAX_IMAGE_HEIGHT];

// Image information
int16_t imageWidth = 0;
int16_t imageHeight = 0;
uint16_t TRANSPARENT_COLOR = 0xFFFF;  // White is used as transparent and will not be drawn on display

// blinking variables for settings/adjustments
boolean blinking = true;
unsigned long previousBlinking = 0;        // msec timestamp
const unsigned long periodBlinking = 500;  // half second on/off = 1 sec frequency

// Setup RTC
RTC_DS1307 RTC;  // actual RTC type in use is DS3231

double observer_lat = 45.0;   // Minneapolis initial preset if none stored in NVRAM
double observer_lon = -93.0;  // West
int8_t NVlat = 0;             // lat to/from NVRAM (full degrees)
int8_t NVlon = 0;             // lon to/from NVRAM

// Posix Timezone rules
// got these definitions from https://support.cyberdata.net/portal/en/kb/articles/010d63c0cfce3676151e1f2d5442e311
// don't take all entries of this webpage for granted, I found a few entries needing corrections like swaping beween standard and DST, as well as wrong weekend definitions
// Maybe better using this Wikipedia map https://upload.wikimedia.org/wikipedia/commons/9/94/Timezones2008_UTC-9_gray.png
// Posix Time Zone format: Mm.n.d/time
//    Mm: The month (1-12).
//    n: The week number (1-5) (use 5 for last week).
//    d: The day of the week (0 for Sunday, 6 for Saturday).
//    time: The time in 24-hour format (e.g., 2:00:00, can be abbreviated).

const uint8_t TZcolumns = 4;
const char *const TZdata[][TZcolumns] = {
  { "Europe", "CET-1CEST,M3.5.0/2:00:00,M10.5.0/2:00:00", "CET", "CEST" },  // Central European Time, Central European Summer Time
  { "UK", "GMT0BST,M3.5.0/2:00:00,M10.5.0/2:00:00", "GMT", "BST" },         // UK GMT, British Summer Time
  { "UTC", "GMT", "UTC", "UTC" },                                           // UTC
  { "Eastern", "EST5EDT,M3.2.0/2:00:00,M11.1.0/2:00:00", "EST", "EDT" },
  { "Central", "CST6CDT,M3.2.0/2:00:00,M11.1.0/2:00:00", "CST", "CDT" },
  { "Mountain", "MST7MDT,M3.2.0/2:00:00,M11.1.0/2:00:00", "MST", "MDT" },
  { "Arizona", "MST7", "MST", "MST" },
  { "Pacific", "PST8PDT,M3.2.0/2:00:00,M11.1.0/2:00:00", "PST", "PDT" },
  { "Alaska", "ASKT9AKDT,M3.2.0/2:00:00,M11.1.0/2:00:00", "AKST", "AKDT" },
  { "Hawaii", "HST10", "HST", "HST" },
};
const uint8_t TZrows = sizeof(TZdata) / sizeof(TZdata[0]);  // get number of rows from TZdata array
uint8_t TZselect = 4;                                       // default TZ US-Central
char TZstr[] = "xxxxxxxxxxxx";                              // display time zone
char TZstrO[] = "xxxxxxxxxxxx";                             // remember previous value to erase pixels before writing new data

time_t UTCsec;                    // Current UTC time in epoch (secs since 1/1/1970)
char thisTime[] = "00:00 xxxx";   // display format time/date/sdt/dst
char thisTimeO[] = "00:00 xxxx";  // display format time/date previous string
char thisDate[] = "00/00/00 ";
char thisDateO[] = "00/00/00 ";
tm ltz;  // Local Time Zone Struct

// Menu variables
byte menu = 0;

// temp all purpose
int temp = 0;

// moonphase variables
bool phaseNow = true;         // trigger phase calculation immediately during settings changes, calculate only once a minute during normal operation
bool MoonCalcDone = false;    // prevent recaclulation once done while "seconds()" still showing zero
bool updatedHWclock = false;  // prevent HW clock update once completed every hour while minute() = 0
int phaseRaw = 0;             // Phase 0 - 1199 0=new, 300=1stQ, 600=full, 900=lastQ
double limbAngle;             // Illuminated limb angle
char limbAngleStr[24];
char limbAngleStrO[24];
char obsLat[] = "Lat:-xxx.x";  // Observer Lat printable string
char obsLatO[] = "Lat:-xxx.x";
char obsLon[] = "Lon:-xxx.x";  // Observer Lon printable string
char obsLonO[] = "Lon:-xxx.x";
double moonAlt = 0;  // moon Alt/Az, Sun Alt/Az and print variables
char moonAltStr[] = "xxxxxxxxxx";
char moonAltStrO[] = "xxxxxxxxxx";
double moonAz = 0;
char moonAzStr[] = "xxxxxxxxxx";
char moonAzStrO[] = "xxxxxxxxxx";
double sunAlt = 0;
char sunAltStr[] = "xxxxxxxxxx";
char sunAltStrO[] = "xxxxxxxxxx";
double sunAz = 0;
char sunAzStr[] = "xxxxxxxxxx";
char sunAzStrO[] = "xxxxxxxxxx";
char tmp2str[] = "***********************";
char tmp2strO[] = "***********************";
char SDfilnam[] = "xxxxxx.png";  // moon phase image filename 0.png to 39.png
char SDfilnamO[] = "xxxxxx.png";
time_t NextMoonUTCO = 0;
time_t moonPhaseUTC[240];  // moon phases (new, 1stQ, full, 3rdQ) for ~5 years
int moonPhaseInd[240];
time_t seasonUTC[24];  // 4 Seasons per year for 5 years
time_t NextMoonUTC[4];
int NextMoonInd[4];
time_t BkMoon = 0;   // next black moon (second new moon in a month)
time_t BlMoonM = 0;  // next monthly Full moon (second full moon in a month)
time_t BlMoonS = 0;  // next seasonal blue moon (third full moon of four in a season)
char BkMoonStr[22];  // Blue/Black Moon print Time/Date and old strings for updates
char BkMoonStrO[22];
char BlMoonMStr[22];
char BlMoonMStrO[22];
char BlMoonSStr[22];
char BlMoonSStrO[22];

void setup() {
  Serial.begin(115200);
  Wire.begin();  // I2C
  RTC.begin();   // Clock

  // following line sets the predefined NVRAM default data, only for the first time if the first two bytes are not containing '16'
  if (!((RTC.readnvram(0) == 16) & (RTC.readnvram(1) == 16))) {
    write_NVRAM();  // write default values
  }
  TZselect = RTC.readnvram(2);                               // read RTC NVRAM TZ setting
  NVlat = RTC.readnvram(3);                                  // observer lat
  NVlon = RTC.readnvram(4);                                  //observer lon
  if (TZselect < 0 || TZselect >= TZrows) { TZselect = 1; }  // Plausibility check NVRAM data
  if (NVlat < -90 || NVlat > 90) { NVlat = 0; }
  if (NVlon < -180 || NVlon > 180) { NVlon = 0; }
  observer_lat = NVlat;  // convert observer position from int to double
  observer_lon = NVlon;

  updateClock();                                   // set HW clock from RTC
  GetTimeStr();                                    // Generate time/date strings
  if (year(now()) < 2000 || year(now()) > 2099) {  // initialize RTC if year <2000 or > 2099. We're using 2-Digit year to display dates
    RTC.adjust(DateTime(2000, 1, 1, 0, 0, 0));     // bring date/time into range if out of bounds
    updateClock();
    GetTimeStr();  // set HW clock on corrected time/date in range
  }
  setenv("TZ", TZdata[TZselect][1], 1);  // set timezone to previously defined strings
  tzset();                               // switch TZ active
  menu = 0;                              // set to show idle screen
  phaseNow = true;                       // trigger moon calc immediately, override once a minute callculation interval

  // Encoder Setup
  rotaryEncoder.setEncoderType(EncoderType::FLOATING);  // encoder does not have pull-up resistors (in contrast to HAS_PULLUP like on a breakout board)
  rotaryEncoder.setBoundaries(0, 1, false);             // Parameters: lower-limit, upper-limit, false=no-wrap - readings are: 0=step left, 1=step right
  rotaryEncoder.onTurned(RotaryEnc);                    // pointer to handling routine
  rotaryEncoder.begin(true);                            // no need for any code in loop()
  pinMode(BtnPin, INPUT_PULLUP);                        // We're handling the button separately, outside of the Encoder Library

  // Init both displays
  pinMode(DispText_CS, OUTPUT);
  pinMode(DispMoon_CS, OUTPUT);
  digitalWrite(DispText_CS, 0);  // CS is Low Active
  digitalWrite(DispMoon_CS, 0);
  tft.begin();
  tft.setRotation(0);  // 0 = portrait, 1 = Landscape, 2 = portrait vert flip, 3 = landscape vert flip
  tft.setTextSize(0);
  tft.fillScreen(TFT_BLACK);
  tft.setFreeFont(FONT1);
  tft.setTextColor(TFT_COLOR1);  // adjust color matching the aged sky background

  // deselct both
  digitalWrite(DispText_CS, 1);
  digitalWrite(DispMoon_CS, 1);

  // Initialize SD card on separate SPI
  SPI.begin(SCLK, MISO, MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failed!");
    return;
  }
}

void loop() {
  if (digitalRead(BtnPin) == LOW && SelBtnOld == false && millis() - BtnRelease > 50) { SelClick(); }  // Press Btn - fork to subroutine after releasing, debounce when released
  if (digitalRead(BtnPin) == HIGH && SelBtnOld == true) {                                              // Reset button status if button is not pressed and old status = pressed
    SelBtnOld = false;
    BtnRelease = millis();  // remember when button was released and de-bounce for 50ms
  }
  if ((minute(now()) == 0 && updatedHWclock == false)) {  // pull RTC time and update HW clock once an hour
    updateClock();                                        // Set HW clock
    updatedHWclock = true;                                // block update once done every hour at minute = 0
  }
  if (minute(now()) != 0) { updatedHWclock = false; }  // reset HW-clock update inhibit

  // 500ms timer for the setup selector, UTC unixtime update, blinking box during settings
  if (millis() - previousBlinking >= periodBlinking) {                                                               // this loop runs every 500 ms
    previousBlinking = millis();                                                                                     // save the last time you blinked
    blinking = !blinking;                                                                                            // switch setting indicator on/off
    UTCsec = DateTime(year(now()), month(now()), day(now()), hour(now()), minute(now()), second(now())).unixtime();  // update UTCsec from HW clock on regular basis
    if (menu != 0) {                                                                                                 // update text parts instant during setting changes
      if (blinking) {                                                                                                // make settings box blink
        TFT_BOXES = TFT_COLOR1;
      } else {
        TFT_BOXES = TFT_BLACK;
      }
      updateTxtDisplay();
    }
  }
  if ((second(now()) == 0 && MoonCalcDone == false) || phaseNow) {  //calculate moon phase only once a minute, override with phaseNow = true during setting changes
    computePhases(year(now()));                                     // calculate moon phases new 1stQ full lastQ for 60 months
    computeSeasons(year(now()));                                    // calculate seasons for 4 years
    FindBlackMoon();                                                // find next black moon (second new moon in a month)
    FindBlueMoonM();                                                // Find next monthly blue moon (second full moon in a month)
    FindBlueMoonS();                                                // Find next seasonal blue moon (3rd full monn in a season with four full moons)

    // Calculate moon illumination angle
    limbAngle = calcMoonIlluminatedLimbAngle(UTCsec, observer_lat, observer_lon);
    Serial.print("Angle: ");
    Serial.print(limbAngle);
    Serial.println("°");  // uncomment of the used font has a degree symbol
    MoonCalcDone = true;
    phaseNow = false;
    // shift phaseRaw by 15 to adjust image display starting 15 (phaseRaw) before until 15 after actual event.
    // PhaseRaw is a number 0-1199 where 0=New, 300=1stQ, 600=Full, 900=LastQ
    // E.g: full moon is at phaseRaw of 600 and the image "full Moon" (#20) should show between 585 and 615, (40 images, 30 steps phaseRaw = 1/40 of 1200)
    phaseRaw += 15;
    if (phaseRaw > 1199) { phaseRaw -= 1200; }   // wrap if result is above 1199
    int filnum = map(phaseRaw, 0, 1199, 0, 40);  // file index 0 to 39
    if (filnum > 39) { filnum = 0; }             // stay in bounds

    LoadMoonFile(filnum);                // check PNG file index number and load new file from SD if needed
    int tmp = 90;                        // preset image rotation to 90 degrees
    if (filnum > 20) { tmp = -90; }      // limb Angle calculation always indicates bright side regardless if waxing or waning, flip rotation when waning.
                                         // Traditional moon phase images assume illuminated part moving right to left (northern hemisphere)
    rotateImage(360 - limbAngle - tmp);  // Rotate image by X degrees (float angle),
                                         // PNGs showing bright limb at 90 DegrLimbAngle calculation is starting on top going CCW, we inverse CW orintation fo easier reading

    // Display the rotated image on LCD in portrait format
    int centerX = (tft.width() - imageWidth) / 2;
    int centerY = (tft.height() - imageHeight) / 2;
    centerY -= 65;  // shift moon 65 pixels up from middle
    displayRotatedImage(centerX, centerY);


    displayMoonSunPos();  // calculate moon and sun Alt/Az
    updateTxtDisplay();   // update text portion
  }
  if (second(now()) != 0 && MoonCalcDone) { MoonCalcDone = false; }  // flag to trigger recalculation next minute
}

void FindBlackMoon() {                // mark monthly blue moon if two dates are in same month
  for (int i = 4; i < 240; i += 4) {  // moon phase list new moon = 0, start at second new moon in list
                                      //  Serial.print(month(moonPhaseUTC[i]));  Serial.print(" "); Serial.print(month(moonPhaseUTC[i - 4])); Serial.print(" "); Serial.print(month(moonPhaseUTC[i])-month(moonPhaseUTC[i-4]));Serial.print(" "); printUnix(moonPhaseUTC[i]);
    if (month(moonPhaseUTC[i]) == month(moonPhaseUTC[i - 4])) {
      BkMoon = moonPhaseUTC[i];
      //      Serial.print("BkMoon:");Serial.print(BkMoon);Serial.print(" UTCsec:");Serial.print(UTCsec);Serial.print(" i:");Serial.print(i);Serial.print(" Dif:");Serial.println(BkMoon-UTCsec);
      if (BkMoon > UTCsec) { return; }  // exit loop if date is in the future
    }
  }
}

void FindBlueMoonM() {                // mark monthly blue moon if two dates are in same month
  for (int i = 6; i < 240; i += 4) {  // moon phase list full moon = 2, start at second full moon in list
    if (month(moonPhaseUTC[i]) == month(moonPhaseUTC[i - 4])) {
      BlMoonM = moonPhaseUTC[i];
      if (BlMoonM > UTCsec) { return; }  // exit loop if date is in the future
    }
  }
}

void FindBlueMoonS() {            // mark third full moon in a season if the season has four full moons
  int k = 0;                      // keep track number of full moons in a season
  int l = 0;                      // 3rd full moon pointer into list of full moons
  for (int j = 1; j < 24; j++) {  // walk through  season dates
    for (int i = 2; i < 240; i += 4) {
      if (moonPhaseUTC[i] > seasonUTC[j - 1] && moonPhaseUTC[i] < seasonUTC[j]) {  // check how many Fmoon[] entries being inside season
        k++;
        if (k == 3) { l = i; }  // remember the third entry
      }
    }
    if (k > 3) {  // if there are four full moons in a season, use the third as blue moon
      BlMoonS = moonPhaseUTC[l];
      if (BlMoonS > UTCsec) { return; }  // exit loop if date is in the future
    }
    k = 0;
  }
}

void convUnix(time_t locUTC) {  // get local time based on timezone, STD/DST and populate 'thisTime' 'thisDate'
  localtime_r(&locUTC, &ltz);   // UTC to TZ struct
  sprintf(thisTime, "%02d:%02d", ltz.tm_hour, ltz.tm_min);
  sprintf(thisDate, "%02d/%02d/%02d", ltz.tm_mon + 1, ltz.tm_mday, ltz.tm_year - 100);
}

void displayMoonSunPos() {
  digitalWrite(DispText_CS, 1);
  digitalWrite(DispMoon_CS, 0);
  int centerX = tft.width() / 2;
  int centerY = tft.height() / 2;
  centerY += 90;                              // shift ellipsis down 90 pixels from middle
  tft.fillRect(0, 180, 240, 140, TFT_BLACK);  // x,y,w,h,color erase bottom half
  tft.drawCircle(centerX, centerY, 10, TFT_COLOR1);
  updateText("", "E", centerX - 5, centerY + 4);  // old string, new string, x-pos, y-pos

  tft.drawEllipse(centerX, centerY, 100, 50, TFT_COLOR1);  // draw orbit

  int MposX = centerX + 90 * cos(degToRad(moonAz + 90));  // calculate moon position on inside of ellipse (add 90 degrees to start at bottom)
  int MposY = centerY + 40 * sin(degToRad(moonAz + 90));
  updateText("", "M", MposX - 5, MposY + 5);              // old string, new string, x-pos, y-pos
  int SposX = centerX + 110 * cos(degToRad(sunAz + 90));  // calculate sun position on outside of ellipse (add 90 degrees to start at bottom)
  int SposY = centerY + 60 * sin(degToRad(sunAz + 90));
  updateText("", "S", SposX - 5, SposY + 5);             // old string, new string, x-pos, y-pos
  tft.drawLine(MposX, MposY, SposX, SposY, TFT_COLOR1);  // draw line between sun and moon
  digitalWrite(DispText_CS, 1);
  digitalWrite(DispMoon_CS, 1);
}

void computePhases(int cyear) {
  int i = 0;
  int k = getCycleEstimate(cyear, 0);  // find first cycle for this year
  float PhaseL = 0;
  for (int i = -1; i < 58; i++) {  // iterate through previous and next 57 cycles
    moonPhaseUTC[(i + 1) * 4] = getPhaseDate(k + i, 0);
    moonPhaseInd[(i + 1) * 4] = 0;  // 0 = New Moon
    moonPhaseUTC[(i + 1) * 4 + 1] = getPhaseDate(k + i, .25);
    moonPhaseInd[(i + 1) * 4 + 1] = 1;  // 1 = First Quarter
    moonPhaseUTC[(i + 1) * 4 + 2] = getPhaseDate(k + i, .5);
    moonPhaseInd[(i + 1) * 4 + 2] = 2;  // 2 = Full Moon
    moonPhaseUTC[(i + 1) * 4 + 3] = getPhaseDate(k + i, .75);
    moonPhaseInd[(i + 1) * 4 + 3] = 3;  // 3 = Last Quarter
  }
  for (i = 0; i < 240; i++) {  // sifting through all moon phase entries until finding the current quarter
    if (moonPhaseUTC[i] > UTCsec) {
      long iPhase = moonPhaseUTC[i] - moonPhaseUTC[i - 1];  // get length of current Moon Phase Quarter
      long uPhase = UTCsec - moonPhaseUTC[i - 1];           // how far are we in the current quarter
      switch (moonPhaseInd[i - 1]) {                        // indicates if we are in the phase after new,1stQ,full,3edQ
        case 0:
          phaseRaw = map(uPhase, 0, iPhase, 0, 299);  // after new moon
          break;
        case 1:
          phaseRaw = map(uPhase, 0, iPhase, 300, 599);  // after 1stQ
          break;
        case 2:
          phaseRaw = map(uPhase, 0, iPhase, 600, 899);  // after full
          break;
        case 3:
          phaseRaw = map(uPhase, 0, iPhase, 900, 1199);  // after 3rdQ
          break;
      }
      for (k = 0; k < 4; k++) {  // collect next four phase dates
        NextMoonUTC[k] = moonPhaseUTC[i + k];
        NextMoonInd[k] = moonPhaseInd[i + k];
      }
      return;  // exit, no need to continue through remaining entries
    }
  }
}

void computeSeasons(int cyear) {                       // compute seasons for 5 years starting at previous year
  time_t UTCseason = 0;                                // Unix Epoch
  char thisSeason[] = "0000-00-00 - 00:00:00";         // placeholder for display output
  char FormatTD[] = "%4d-%02d-%02d - %02d:%02d:%02d";  // output format
  int idx = 0;                                         // index into seasonUTC[]
  for (int Y = cyear - 1; Y < cyear + 5; Y++) {        // start previous year
    double M = (Y - 2000) / 1000.0;                    // convert AD year to millennia, from 2000 AD.;
    double T[4], W[4], L[4], S[4], JD[4];              // Temp variables
    // perturbation constants (24 elements)
    double A[] = { 485, 203, 199, 182, 156, 136, 77, 74, 70, 58, 52, 50, 45, 44, 29, 18, 17, 16, 14, 12, 12, 12, 9, 8 };
    double B[] = { 324.96, 337.23, 342.08, 27.85, 73.14, 171.52, 222.54, 296.72, 243.58, 119.81, 297.17, 21.02, 247.54, 325.15, 60.93, 155.12, 288.79, 198.04, 199.76, 95.39, 287.11, 320.81, 227.73, 15.45 };
    double C[] = { 1934.136, 32964.467, 20.186, 445267.112, 45036.886, 22518.443, 65928.934, 3034.906, 9037.513, 33718.147, 150.678, 2281.226, 29929.562, 31555.956, 4443.417, 67555.328, 4562.452, 62894.029, 31436.921, 14577.848, 31931.756, 34777.259, 1222.114, 16859.074 };
    double JDME[4];                                                                                                // initial rough position
    JDME[0] = 2451623.80984 + 365242.37404 * M + 0.05169 * pow(M, 2) - 0.00411 * pow(M, 3) - 0.00057 * pow(M, 4);  // March Equinox Base
    JDME[1] = 2451716.56767 + 365241.62603 * M + 0.00325 * pow(M, 2) + 0.00888 * pow(M, 3) - 0.00030 * pow(M, 4);  // June Solstice Base
    JDME[2] = 2451810.21715 + 365242.01767 * M - 0.11575 * pow(M, 2) + 0.00337 * pow(M, 3) + 0.00078 * pow(M, 4);  // September Equinox Base
    JDME[3] = 2451900.05952 + 365242.74049 * M - 0.06223 * pow(M, 2) - 0.00823 * pow(M, 3) + 0.00032 * pow(M, 4);  // December Solstice Base
    for (int i = 0; i < 4; ++i) {
      T[i] = (JDME[i] - 2451545.0) / 36525;                                          // Julian centuries from 2000 (of equ/sol).
      W[i] = 35999.373 * T[i] - 2.47;                                                // degrees
      L[i] = 1 + 0.0334 * cos(W[i] * PI / 180) + 0.0007 * cos(2 * W[i] * PI / 180);  // Lambda
      S[i] = 0;                                                                      //
      for (int j = 0; j < 24; j++) {                                                 //
        S[i] += A[j] * cos((B[j] + C[j] * T[i]) * PI / 180);                         // compute perturbation
      }                                                                              //
      JD[i] = JDME[i] + 0.00001 * S[i] / L[i];                                       // The final result in Julian Dynamical Days.
      UTCseason = julianDateToUnix(JD[i]);
      seasonUTC[idx] = UTCseason;
      idx += 1;
    }
  }
}

unsigned long julianDateToUnix(double julianDate) {  // Julian day for Unix epoch
  const double UNIX_EPOCH_JULIAN_DAY = 2440587.5;
  time_t unixTime = (long)((julianDate - UNIX_EPOCH_JULIAN_DAY) * 86400);  // Calculate the difference in days and convert to seconds
  return unixTime;
}

double mod360(int f) {
  int t = f % 360;
  if (t < 0) t += 360;
  return t;
}

double getCycleEstimate(int year, int month) {
  double yearfrac = (month * 30 + 15) / 365.0;      //Estimate fraction of year
  double k = 12.3685 * ((year + yearfrac) - 2000);  //49.2
  k = floor(k);
  return k;
}

unsigned long getPhaseDate(double cycle, double phase) {  //From Meeus Astronomical Algorithms - ch49
  double correction = 0;
  double k = cycle + phase;
  double toRad = PI / 180;
  double T = k / 1236.85;                                                                                                              //49.3
  double JDE = 2451550.09766 + 29.530588861 * k + 0.00015437 * T * T - 0.000000150 * T * T * T + 0.00000000073 * T * T * T * T;        //49.1
  double E = 1 - 0.002516 * T - 0.0000074 * T * T;                                                                                     //47.6
  double M = mod360(2.5534 + 29.10535670 * k - 0.0000014 * T * T - 0.00000011 * T * T * T) * toRad;                                    //49.4
  double Mp = mod360(201.5643 + 385.81693528 * k + 0.0107582 * T * T + 0.00001238 * T * T * T - 0.000000058 * T * T * T * T) * toRad;  //49.5
  double F = mod360(160.7108 + 390.67050284 * k - 0.0016118 * T * T - 0.00000227 * T * T * T + 0.000000011 * T * T * T * T) * toRad;   //49.6
  double Om = mod360(124.7746 - 1.56375588 * k + 0.0020672 * T * T + 0.00000215 * T * T * T) * toRad;                                  //49.7

  //P351-352
  double A1 = mod360(299.77 + 0.107408 * k - 0.009173 * T * T) * toRad;
  double A2 = mod360(251.88 + 0.016321 * k) * toRad;
  double A3 = mod360(251.83 + 26.651886 * k) * toRad;
  double A4 = mod360(349.42 + 36.412478 * k) * toRad;
  double A5 = mod360(84.66 + 18.206239 * k) * toRad;
  double A6 = mod360(141.74 + 53.303771 * k) * toRad;
  double A7 = mod360(207.14 + 2.453732 * k) * toRad;
  double A8 = mod360(154.84 + 7.306860 * k) * toRad;
  double A9 = mod360(34.52 + 27.261239 * k) * toRad;
  double A10 = mod360(207.19 + 0.121824 * k) * toRad;
  double A11 = mod360(291.34 + 1.844379 * k) * toRad;
  double A12 = mod360(161.72 + 24.198154 * k) * toRad;
  double A13 = mod360(239.56 + 25.513099 * k) * toRad;
  double A14 = mod360(331.55 + 3.592518 * k) * toRad;

  if (phase == 0) {
    correction = 0.00002 * sin(4 * Mp) + -0.00002 * sin(3 * Mp + M) + -0.00002 * sin(Mp - M - 2 * F) + 0.00003 * sin(Mp - M + 2 * F) + -0.00003 * sin(Mp + M + 2 * F) + 0.00003 * sin(2 * Mp + 2 * F) + 0.00003 * sin(Mp + M - 2 * F) + 0.00004 * sin(3 * M) + 0.00004 * sin(2 * Mp - 2 * F) + -0.00007 * sin(Mp + 2 * M) + -0.00017 * sin(Om) + -0.00024 * E * sin(2 * Mp - M) + 0.00038 * E * sin(M - 2 * F) + 0.00042 * E * sin(M + 2 * F) + -0.00042 * sin(3 * Mp) + 0.00056 * E * sin(2 * Mp + M) + -0.00057 * sin(Mp + 2 * F) + -0.00111 * sin(Mp - 2 * F) + 0.00208 * E * E * sin(2 * M) + -0.00514 * E * sin(Mp + M) + 0.00739 * E * sin(Mp - M) + 0.01039 * sin(2 * F) + 0.01608 * sin(2 * Mp) + 0.17241 * E * sin(M) + -0.40720 * sin(Mp);
  } else if ((phase == 0.25) || (phase == 0.75)) {
    correction = -0.00002 * sin(3 * Mp + M) + 0.00002 * sin(Mp - M + 2 * F) + 0.00002 * sin(2 * Mp - 2 * F) + 0.00003 * sin(3 * M) + 0.00003 * sin(Mp + M - 2 * F) + 0.00004 * sin(Mp - 2 * M) + -0.00004 * sin(Mp + M + 2 * F) + 0.00004 * sin(2 * Mp + 2 * F) + -0.00005 * sin(Mp - M - 2 * F) + -0.00017 * sin(Om) + 0.00027 * E * sin(2 * Mp + M) + -0.00028 * E * E * sin(Mp + 2 * M) + 0.00032 * E * sin(M - 2 * F) + 0.00032 * E * sin(M + 2 * F) + -0.00034 * E * sin(2 * Mp - M) + -0.00040 * sin(3 * Mp) + -0.00070 * sin(Mp + 2 * F) + -0.00180 * sin(Mp - 2 * F) + 0.00204 * E * E * sin(2 * M) + 0.00454 * E * sin(Mp - M) + 0.00804 * sin(2 * F) + 0.00862 * sin(2 * Mp) + -0.01183 * E * sin(Mp + M) + 0.17172 * E * sin(M) + -0.62801 * sin(Mp);
    double W = 0.00306 - 0.00038 * E * cos(M) + 0.00026 * cos(Mp) - 0.00002 * cos(Mp - M) + 0.00002 * cos(Mp + M) + 0.00002 * cos(2 * F);
    if (phase == 0.25) {
      correction += W;
    } else {
      correction -= W;
    }
  } else if (phase == 0.5) {
    correction = 0.00002 * sin(4 * Mp) + -0.00002 * sin(3 * Mp + M) + -0.00002 * sin(Mp - M - 2 * F) + 0.00003 * sin(Mp - M + 2 * F) + -0.00003 * sin(Mp + M + 2 * F) + 0.00003 * sin(2 * Mp + 2 * F) + 0.00003 * sin(Mp + M - 2 * F) + 0.00004 * sin(3 * M) + 0.00004 * sin(2 * Mp - 2 * F) + -0.00007 * sin(Mp + 2 * M) + -0.00017 * sin(Om) + -0.00024 * E * sin(2 * Mp - M) + 0.00038 * E * sin(M - 2 * F) + 0.00042 * E * sin(M + 2 * F) + -0.00042 * sin(3 * Mp) + 0.00056 * E * sin(2 * Mp + M) + -0.00057 * sin(Mp + 2 * F) + -0.00111 * sin(Mp - 2 * F) + 0.00209 * E * E * sin(2 * M) + -0.00514 * E * sin(Mp + M) + 0.00734 * E * sin(Mp - M) + 0.01043 * sin(2 * F) + 0.01614 * sin(2 * Mp) + 0.17302 * E * sin(M) + -0.40614 * sin(Mp);
  }
  JDE += correction;

  //Additional corrections P 252
  correction = 0.000325 * sin(A1) + 0.000165 * sin(A2) + 0.000164 * sin(A3) + 0.000126 * sin(A4) + 0.000110 * sin(A5) + 0.000062 * sin(A6) + 0.000060 * sin(A7) + 0.000056 * sin(A8) + 0.000047 * sin(A9) + 0.000042 * sin(A10) + 0.000040 * sin(A11) + 0.000037 * sin(A12) + 0.000035 * sin(A13) + 0.000023 * sin(A14);
  JDE += correction;

  // convert Julian to Epoch
  unsigned long UTCdate = julianDateToUnix(JDE);

  return UTCdate;
}

void LoadMoonFile(int idx) {  // check if file ID changed and loading is needed
  char stridx[] = "xxxxxx";
  itoa(idx, stridx, 10);  // convert integer idx to string, base 10
  strcpy(SDfilnam, "/");
  strcat(SDfilnam, stridx);
  strcat(SDfilnam, ".png");

  if (strcmp(SDfilnamO, SDfilnam)) {  // check if we need to load different file
    strcpy(SDfilnamO, SDfilnam);      // remember loaded filename
    freeImageBuffers();
    Serial.print("Loading ");
    Serial.println(SDfilnam);

    if (loadPngFromSD(SDfilnam)) {  // Load Moon PNG from SD card
      Serial.println("PNG loaded successfully");
    } else {
      Serial.println("Failed to load PNG");
    }
  }
}

// calculate angle of observed illuminated limb based on observer lat/lon
double calcMoonIlluminatedLimbAngle(time_t time, double observer_lat, double observer_lon) {
  struct tm *tm_info = gmtime(&time);

  // Calculate Julian Date
  double jd = julianDate(tm_info->tm_year + 1900, tm_info->tm_mon + 1,
                         tm_info->tm_mday, tm_info->tm_hour,
                         tm_info->tm_min, tm_info->tm_sec);

  // Compute moon and sun positions
  double moonRA, moonDec, moonDist;
  calcMoonPosition(time, &moonRA, &moonDec, &moonDist);
  Serial.print("MoonRA:");
  Serial.print(moonRA, 1);
  Serial.print(" MoonDec:");
  Serial.println(moonDec, 1);
  double sunRA, sunDec;
  calcSunPosition(time, &sunRA, &sunDec);
  Serial.print("SunRA:");
  Serial.print(sunRA, 1);
  Serial.print(" SunDec:");
  Serial.println(sunDec, 1);

  // Calculate Local Sidereal Time
  double T = (jd - 2451545.0) / 36525.0;
  double theta0 = 280.46061837 + 360.98564736629 * (jd - 2451545.0) + 0.000387933 * T * T - T * T * T / 38710000.0;
  theta0 = fmod(theta0, 360.0);
  double LST = fmod(theta0 + observer_lon, 360.0);

  // Convert to horizontal coordinates
  equatorialToHorizontal(moonRA, moonDec, LST, observer_lat, &moonAz, &moonAlt);
  //Serial.print("MoonAz:");Serial.print(moonAz,1);Serial.print(" MoonAlt:");Serial.println(moonAlt,1);
  //double sunAz, sunAlt;
  equatorialToHorizontal(sunRA, sunDec, LST, observer_lat, &sunAz, &sunAlt);
  //Serial.print("SunAz:");Serial.print(sunAz,1);Serial.print(" SunAlt:");Serial.println(sunAlt,1);

  // Compute the position angle of the bright limb (relative to moon's north)
  double y = cos(degToRad(sunDec)) * sin(degToRad(sunRA - moonRA));
  double x = sin(degToRad(sunDec)) * cos(degToRad(moonDec)) - cos(degToRad(sunDec)) * sin(degToRad(moonDec)) * cos(degToRad(sunRA - moonRA));
  double chi = radToDeg(atan2(y, x));

  // Adjust for the observer's view (parallactic angle)
  double parallacticAngle = 0.0;
  if (cos(degToRad(moonDec)) != 0.0) {
    double sinP = sin(degToRad(LST - moonRA));
    double cosP = tan(degToRad(observer_lat)) * cos(degToRad(moonDec)) - sin(degToRad(moonDec)) * cos(degToRad(LST - moonRA));
    parallacticAngle = radToDeg(atan2(sinP, cosP));
  }

  // Calculate the position angle of the bright limb (position Angle minus parallactic angle) and keep in 0-360 bounds
  double brightLimbAngle = fmod(chi - parallacticAngle, 360.0);

  // Normalize to 0-360 degrees
  if (brightLimbAngle < 0) brightLimbAngle += 360.0;

  return brightLimbAngle;
}

void calcMoonPosition(time_t time, double *RA, double *Dec, double *distance) {
  struct tm *tm_info = gmtime(&time);

  // Calculate Julian Date
  double jd = julianDate(tm_info->tm_year + 1900, tm_info->tm_mon + 1,
                         tm_info->tm_mday, tm_info->tm_hour,
                         tm_info->tm_min, tm_info->tm_sec);

  // Time in Julian centuries since J2000.0
  double T = (jd - 2451545.0) / 36525.0;

  // Moon's mean longitude
  double L = 218.316 + 13.176396 * (jd - 2451545.0);
  L = fmod(L, 360.0);

  // Moon's mean elongation
  double D = 297.8502 + 445267.1115 * T - 0.0016300 * T * T + T * T * T / 545868.0 - T * T * T * T / 113065000.0;
  D = fmod(D, 360.0);

  // Sun's mean anomaly
  double M = 357.5291 + 35999.0503 * T - 0.0001559 * T * T - 0.00000048 * T * T * T;
  M = fmod(M, 360.0);

  // Moon's mean anomaly
  double M_moon = 134.9634 + 477198.8675 * T + 0.0089970 * T * T + T * T * T / 69699.0 - T * T * T * T / 14712000.0;
  M_moon = fmod(M_moon, 360.0);

  // Moon's argument of latitude
  double F = 93.2720 + 483202.0175 * T - 0.0034029 * T * T - T * T * T / 3526000.0 + T * T * T * T / 863310000.0;
  F = fmod(F, 360.0);

  // Longitude of the ascending node of the lunar orbit
  double Omega = 125.0440 - 1934.1360 * T + 0.0020680 * T * T + T * T * T / 450000.0;
  Omega = fmod(Omega, 360.0);

  // Calculate geocentric longitude and latitude
  // This is a simplified model - in production would use more terms for higher accuracy, no need here
  double lambda = L + 6.289 * sin(degToRad(M_moon)) + 1.274 * sin(degToRad(2 * D - M_moon)) + 0.658 * sin(degToRad(2 * D)) + 0.214 * sin(degToRad(2 * M_moon));

  double beta = 5.128 * sin(degToRad(F)) + 0.281 * sin(degToRad(M_moon + F)) + 0.277 * sin(degToRad(M_moon - F));

  // Distance in Earth radii
  *distance = 60.36 - 3.27 * cos(degToRad(M_moon)) - 0.57 * cos(degToRad(2 * D - M_moon)) - 0.34 * cos(degToRad(2 * D)) - 0.11 * cos(degToRad(2 * M_moon));

  // Convert ecliptic coordinates to equatorial coordinates
  double epsilon = 23.439 - 0.0000004 * T;  // Obliquity of the ecliptic
  double lambda_rad = degToRad(lambda);
  double beta_rad = degToRad(beta);
  double epsilon_rad = degToRad(epsilon);

  double ra = atan2(sin(lambda_rad) * cos(epsilon_rad) - tan(beta_rad) * sin(epsilon_rad), cos(lambda_rad));
  *RA = fmod(radToDeg(ra), 360.0);
  if (*RA < 0) *RA += 360.0;

  double dec = asin(sin(beta_rad) * cos(epsilon_rad) + cos(beta_rad) * sin(epsilon_rad) * sin(lambda_rad));
  *Dec = radToDeg(dec);
}

void calcSunPosition(time_t time, double *RA, double *Dec) {
  struct tm *tm_info = gmtime(&time);

  // Calculate Julian Date
  double jd = julianDate(tm_info->tm_year + 1900, tm_info->tm_mon + 1,
                         tm_info->tm_mday, tm_info->tm_hour,
                         tm_info->tm_min, tm_info->tm_sec);

  // Time in Julian centuries since J2000.0
  double T = (jd - 2451545.0) / 36525.0;

  // Sun's mean longitude
  double L0 = 280.46646 + 36000.76983 * T + 0.0003032 * T * T;
  L0 = fmod(L0, 360.0);

  // Sun's mean anomaly
  double M = 357.52911 + 35999.05029 * T - 0.0001537 * T * T;
  M = fmod(M, 360.0);

  // Equation of center
  double C = (1.914602 - 0.004817 * T - 0.000014 * T * T) * sin(degToRad(M)) + (0.019993 - 0.000101 * T) * sin(degToRad(2 * M)) + 0.000289 * sin(degToRad(3 * M));

  // True longitude
  double lambda = L0 + C;

  // Obliquity of the ecliptic
  double epsilon = 23.439 - 0.0000004 * T;

  // Convert ecliptic coordinates to equatorial coordinates
  double lambda_rad = degToRad(lambda);
  double epsilon_rad = degToRad(epsilon);

  double ra = atan2(cos(epsilon_rad) * sin(lambda_rad), cos(lambda_rad));
  *RA = fmod(radToDeg(ra), 360.0);
  if (*RA < 0) *RA += 360.0;

  double dec = asin(sin(epsilon_rad) * sin(lambda_rad));
  *Dec = radToDeg(dec);
}

void equatorialToHorizontal(double RA, double Dec, double LST, double lat, double *azimuth, double *altitude) {
  // Hour angle
  double HA = LST - RA;
  if (HA < 0) HA += 360.0;

  // Convert to radians
  double ha_rad = degToRad(HA);
  double dec_rad = degToRad(Dec);
  double lat_rad = degToRad(lat);

  // Calculate altitude
  double sin_alt = sin(dec_rad) * sin(lat_rad) + cos(dec_rad) * cos(lat_rad) * cos(ha_rad);
  double alt_rad = asin(sin_alt);
  *altitude = radToDeg(alt_rad);

  // Calculate azimuth
  double cos_az = (sin(dec_rad) - sin(lat_rad) * sin_alt) / (cos(lat_rad) * cos(alt_rad));

  // Handle out of range values due to floating point errors
  if (cos_az > 1.0) cos_az = 1.0;
  if (cos_az < -1.0) cos_az = -1.0;

  double az = acos(cos_az);

  // Adjust azimuth based on hour angle
  if (sin(ha_rad) >= 0) az = 2 * M_PI - az;

  *azimuth = radToDeg(az);
}

// Calculate Julian Date from calendar date and time
double julianDate(int year, int month, int day, int hour, int minute, int second) {
  if (month <= 2) {
    year -= 1;
    month += 12;
  }

  int A = floor(year / 100.0);
  int B = 2 - A + floor(A / 4.0);

  double JD = floor(365.25 * (year + 4716)) + floor(30.6001 * (month + 1)) + day + B - 1524.5;
  JD += (hour + minute / 60.0 + second / 3600.0) / 24.0;

  return JD;
}

// Utility functions for angle conversions
double degToRad(double degrees) {
  return degrees * M_PI / 180.0;
}

double radToDeg(double radians) {
  return radians * 180.0 / M_PI;
}

void updateTxtDisplay() {  // Text portion update
  GetTimeStr();
  int i = 0;
  char tmpstr[] = "**********";
  digitalWrite(DispText_CS, 0);  // select text display
  digitalWrite(DispMoon_CS, 1);

  sprintf(limbAngleStr, "Illum Limb Angle:%3.0f", 360.0 - limbAngle);  // Meeus' calculation results angle proceeding CCW from moon north. We reverse displayed angle from CCW to CW for typical reading.
  updateText(limbAngleStrO, limbAngleStr, 5, 13);                      // parameters: old string on display to be deleted, new string to be written, x-pos, y-pos
  strcpy(limbAngleStrO, limbAngleStr);                                 // remember currently displayed string

  updateText("", " Alt   Az", 70, 180);  // Just a static output, nothing to delete
  updateText("", "Moon:", 5, 195);
  sprintf(moonAltStr, "%4.0f", moonAlt);
  updateText(moonAltStrO, moonAltStr, 70, 195);  // moon altitude
  strcpy(moonAltStrO, moonAltStr);

  sprintf(moonAzStr, "%4.0f", moonAz);
  updateText(moonAzStrO, moonAzStr, 125, 195);  // moon azimuth
  strcpy(moonAzStrO, moonAzStr);

  updateText("", "Sun:", 5, 210);
  sprintf(sunAltStr, "%4.0f", sunAlt);
  updateText(sunAltStrO, sunAltStr, 70, 210);  // sun altitude
  strcpy(sunAltStrO, sunAltStr);

  sprintf(sunAzStr, "%4.0f", sunAz);
  updateText(sunAzStrO, sunAzStr, 125, 210);  // sun azimuth
  strcpy(sunAzStrO, sunAzStr);

  updateText(thisDateO, thisDate, 5, 260);  // current local date
  strcpy(thisDateO, thisDate);

  updateText(thisTimeO, thisTime, 115, 260);  // current local time
  strcpy(thisTimeO, thisTime);

  updateText("", "TimeZone:", 5, 280);
  sprintf(TZstr, "%s", TZdata[TZselect][0]);
  updateText(TZstrO, TZstr, 115, 280);  // TZ name
  strcpy(TZstrO, TZstr);

  updateText("", "Observer:", 5, 300);
  sprintf(obsLat, "Lat:%4.0f", observer_lat);
  updateText(obsLatO, obsLat, 5, 315);  // observer latitude
  strcpy(obsLatO, obsLat);

  sprintf(obsLon, "Lon:%4.0f", observer_lon);
  updateText(obsLonO, obsLon, 115, 315);  // observer longitude
  strcpy(obsLonO, obsLon);

  for (i = 0; i < 4; i++) {  // line label for the four next moon cycle dates (new,1stQ,full,3rdQ)
    switch (NextMoonInd[i]) {
      case 0:
        strcpy(tmpstr, " New:");
        break;
      case 1:
        strcpy(tmpstr, "1stQ:");
        break;
      case 2:
        strcpy(tmpstr, "Full:");
        break;
      case 3:
        strcpy(tmpstr, "LstQ:");
        break;
    }

    if (NextMoonUTCO != NextMoonUTC[0]) {       // erase all next moon dates if there was a change in the first UTC timestamp
      tft.fillRect(0, 15, 240, 85, TFT_BLACK);  // coordinates x,y,w,h - color fill of rectangle
      updateText("", "Next Moon: ", 5, 35);     // rewrite header of table
      NextMoonUTCO = NextMoonUTC[0];            // remember first row timestamp
    }

    convUnix(NextMoonUTC[i]);  // print readable timestamps for next moon phase dates
    sprintf(tmp2str, "%s %s %s", tmpstr, thisDate, thisTime);
    updateText(tmp2strO, tmp2str, 5, 50 + i * 15);
  }

  convUnix(BkMoon);  // next black moon
  sprintf(BkMoonStr, " Blk: %s %s", thisDate, thisTime);
  updateText(BkMoonStrO, BkMoonStr, 5, 120);
  strcpy(BkMoonStrO, BkMoonStr);

  convUnix(BlMoonM);  // next monthly blue moon
  sprintf(BlMoonMStr, "Bl-M: %s %s", thisDate, thisTime);
  updateText(BlMoonMStrO, BlMoonMStr, 5, 135);
  strcpy(BlMoonMStrO, BlMoonMStr);

  convUnix(BlMoonS);  // next seasonal blue moon
  sprintf(BlMoonSStr, "Bl-S: %s %s", thisDate, thisTime);
  updateText(BlMoonSStrO, BlMoonSStr, 5, 150);
  strcpy(BlMoonSStrO, BlMoonSStr);

  switch (menu) {  // draw highlighted boxes during settings change
    case 11:       // Settings Month
      tft.drawRect(2, 247, 30, 18, TFT_BOXES);
      break;
    case 12:  // Settings Day
      tft.drawRect(34, 247, 30, 18, TFT_BOXES);
      break;
    case 13:  // Settings Year
      tft.drawRect(66, 247, 30, 18, TFT_BOXES);
      break;
    case 14:  // Settings Hour
      tft.drawRect(112, 247, 30, 18, TFT_BOXES);
      break;
    case 15:  // Settings Minute
      tft.drawRect(144, 247, 30, 18, TFT_BOXES);
      break;
    case 16:  // Settings TimeZone
      tft.drawRect(112, 267, 110, 18, TFT_BOXES);
      break;
    case 17:  // Settings Observer Lat
      tft.drawRect(2, 302, 110, 18, TFT_BOXES);
      break;
    case 18:  // Settings Observer Lon
      tft.drawRect(112, 302, 110, 18, TFT_BOXES);
      break;
  }
  digitalWrite(DispText_CS, 1);  // Deselect displays
  digitalWrite(DispMoon_CS, 1);
}

void updateText(char tberased[20], char tbwritten[20], int xpos, int ypos) {
  tft.setCursor(xpos, ypos);
  tft.setTextColor(TFT_BLACK);  // adjust color overwrite in black
  tft.print(tberased);
  tft.setCursor(xpos, ypos);
  tft.setTextColor(TFT_COLOR1);  // adjust forground color
  tft.print(tbwritten);
}

void updateClock() {
  UTCsec = RTC.now().unixtime();  // Get epoch time from RTC
  setTime(UTCsec);                // set HW clock
}

void updateRTC() {
  setTime(UTCsec);     // set HW clock
  RTC.adjust(UTCsec);  // update RTC
  phaseNow = true;     // trigger moon calc immediately, override once a minute calc interval
}

void GetTimeStr() {            // get local time based on timezone, STD/DST
  localtime_r(&UTCsec, &ltz);  // UTC to TZ struct
  char tmp[] = "xxxx";
  if (ltz.tm_isdst == 1) {
    strcpy(tmp, TZdata[TZselect][3]);  // DST
  } else {
    strcpy(tmp, TZdata[TZselect][2]);  // Standard Time
  }
  sprintf(thisTime, "%02d:%02d %s", ltz.tm_hour, ltz.tm_min, tmp);
  sprintf(thisDate, "%02d/%02d/%02d", ltz.tm_mon + 1, ltz.tm_mday, ltz.tm_year - 100);
  //sprintf(thisWkDst,"Weekday %d DST %d", ltz.tm_wday, );
}

void write_NVRAM(void) {
  NVlat = observer_lat;
  NVlon = observer_lon;
  uint8_t writeData[9] = { 16, 16, TZselect, NVlat, NVlon, 0, 0, 0, 0 };
  RTC.writenvram(0, writeData, 9);
}

void countUp() {  // this void runs on turn right
  switch (menu) {
    case 11:
      temp = month(UTCsec) + 1;  // add one month
      if (temp > 12) {
        temp = 1;
      }
      UTCsec = DateTime(year(UTCsec), temp, day(UTCsec), hour(UTCsec), minute(UTCsec), second(UTCsec)).unixtime();
      //UTCsec = mktime(&);
      break;
    case 12:
      UTCsec += 86400;  // add one day
      break;
    case 13:
      temp = year(UTCsec) + 1;
      if (temp > 2099) { temp = 2099; }
      UTCsec = DateTime(temp, month(UTCsec), day(UTCsec), hour(UTCsec), minute(UTCsec), second(UTCsec)).unixtime();
      break;
    case 14:
      UTCsec += 3600;  // add one hour
      break;
    case 15:
      UTCsec += 60;  // add one minute
      break;
    case 16:
      TZselect += 1;  // next TZ
      if (TZselect > TZrows - 1) { TZselect = 0; }
      setenv("TZ", TZdata[TZselect][1], 1);  // set timezone to previously defined strings
      tzset();                               // switch TZ active
      NextMoonUTCO = 0UL;                    // flag screen update for next moon event dates
      break;
    case 17:
      observer_lat += 1;  // add half a degree lat
      if (observer_lat > 90) { observer_lat -= 180; }
      break;
    case 18:
      observer_lon += 1;  // add half a degree lon
      if (observer_lat > 180) { observer_lat -= 360; }
      break;
    default:
      return;  // return if not in one of the setting modes avoiding change in RTC
      break;
  }
  updateRTC();  // updates only when settings change
}

void countDn() {  // this void runs on turn left
  switch (menu) {
    case 11:
      temp = month(UTCsec) - 1;  // back one month
      if (temp <= 0) {
        temp = 12;
      }
      UTCsec = DateTime(year(UTCsec), temp, day(UTCsec), hour(UTCsec), minute(UTCsec), second(UTCsec)).unixtime();
      break;
    case 12:
      UTCsec -= 86400;  // back one day
      break;
    case 13:
      temp = year(UTCsec) - 1;
      if (temp < 2000) { temp = 2000; }
      UTCsec = DateTime(temp, month(UTCsec), day(UTCsec), hour(UTCsec), minute(UTCsec), second(UTCsec)).unixtime();
      break;
    case 14:
      UTCsec -= 3600;  // back one hour
      break;
    case 15:
      UTCsec -= 60;  // back one minute
      break;
    case 16:
      TZselect -= 1;  // previlous TZ
      if (TZselect >= TZrows) { TZselect = TZrows - 1; }
      setenv("TZ", TZdata[TZselect][1], 1);  // set timezone to previously defined strings
      tzset();                               // switch TZ active
      NextMoonUTCO = 0UL;                    // flag update for next moon event dates
      break;
    case 17:
      observer_lat -= 1;                                // minus half a degree lat
      if (observer_lat < -90) { observer_lat += 180; }  // wrap around
      break;
    case 18:
      observer_lon -= 1;                                 // minus half a degree lon
      if (observer_lat < -180) { observer_lat += 360; }  // wrap around
      break;
    default:
      return;  // return if not in one of the setting modes avoiding change in RTC
      break;
  }
  updateRTC();  // updates only when settings change
}

void SelClick() {                       // duration used for entering settings
  SelBtnOld = true;                     // remember current button status
                                        //  if (!dispActive) { return; }           // avoid changes when display has just turned active
  unsigned long firstPress = millis();  // remember initial press timestamp
  while (digitalRead(BtnPin) == LOW) {
    if (millis() - firstPress > 600) {  // Long click (>600 msec)
      switch (menu) {
        case 0:
          menu = 11;  // Date/Time settings
          break;
        default:
          menu = 0;  // Unused, back to display default
          break;
      }
      //Serial.print("Long Click, Menu: ");
      //Serial.println(menu);
      return;
    }
  }
  digitalWrite(DispText_CS, 0);
  digitalWrite(DispMoon_CS, 1);

  switch (menu) {
    case 11:
      menu = 12;
      tft.drawRect(2, 247, 30, 18, TFT_BLACK);  // clear rectangle Settings Month
      break;
    case 12:
      menu = 13;
      tft.drawRect(34, 247, 30, 18, TFT_BLACK);  // Day
      break;
    case 13:
      menu = 14;
      tft.drawRect(66, 247, 30, 18, TFT_BLACK);  // Year
      break;
    case 14:
      menu = 15;
      tft.drawRect(112, 247, 30, 18, TFT_BLACK);  //Hour
      break;
    case 15:
      menu = 16;
      tft.drawRect(144, 247, 30, 18, TFT_BLACK);  // Minute
      break;
    case 16:
      menu = 17;
      tft.drawRect(112, 267, 110, 18, TFT_BLACK);  // Time Zone
      break;
    case 17:
      menu = 18;
      tft.drawRect(2, 302, 110, 18, TFT_BLACK);  // Lat
      break;
    case 18:
      menu = 0;
      tft.drawRect(112, 302, 110, 18, TFT_BLACK);  // Lon
      write_NVRAM();                               // menu done, save settings
      break;
    default:
      break;
  }
  digitalWrite(DispText_CS, 1);
  digitalWrite(DispMoon_CS, 1);
}

void displayRotatedImage(int x, int y) {  // Display the rotated image on the TFT
  digitalWrite(DispText_CS, 1);
  digitalWrite(DispMoon_CS, 0);
  for (int row = 0; row < imageHeight; row++) {
    if (rotatedImage[row] != NULL) {
      tft.pushImage(x, y + row, imageWidth, 1, rotatedImage[row], TRANSPARENT_COLOR);
    }
  }
  digitalWrite(DispText_CS, 1);
  digitalWrite(DispMoon_CS, 1);
}

void freeImageBuffers() {  // Free memory for both image buffers to load new one
  for (int i = 0; i < MAX_IMAGE_HEIGHT; i++) {
    if (originalImage[i]) free(originalImage[i]);
    if (rotatedImage[i]) free(rotatedImage[i]);
    originalImage[i] = NULL;
    rotatedImage[i] = NULL;
  }
}

void RotaryEnc(long EncPos) {  // front panel encoder
  //dispTimer = millis() + periodDisplay;  // refresh display on-time
  //  if (!dispActive) { return; }           // avoid changes when display has just turned active
  if (EncPos == 0) {
    countDn();
  } else {
    countUp();
  }
}

void pngDraw(PNGDRAW *pDraw) {  // Function to draw PNG to originalImage buffer
  uint16_t lineBuffer[MAX_IMAGE_WIDTH];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);

  // Copy this line to our image buffer if the row exists
  if (pDraw->y < imageHeight && originalImage[pDraw->y] != NULL) {
    for (int x = 0; x < pDraw->iWidth && x < imageWidth; x++) {
      originalImage[pDraw->y][x] = lineBuffer[x];
    }
  }
}

bool allocateImageBuffers() {  // Allocate memory for both image buffers
  // Initialize all pointers to NULL
  for (int i = 0; i < MAX_IMAGE_HEIGHT; i++) {
    originalImage[i] = NULL;
    rotatedImage[i] = NULL;
  }

  // Allocate memory for each row
  for (int i = 0; i < imageHeight; i++) {
    originalImage[i] = (uint16_t *)malloc(imageWidth * sizeof(uint16_t));
    rotatedImage[i] = (uint16_t *)malloc(imageWidth * sizeof(uint16_t));

    if (!originalImage[i] || !rotatedImage[i]) {
      // Memory allocation failed, free already allocated memory
      for (int j = 0; j <= i; j++) {
        if (originalImage[j]) free(originalImage[j]);
        if (rotatedImage[j]) free(rotatedImage[j]);
        originalImage[j] = NULL;
        rotatedImage[j] = NULL;
      }
      return false;
    }

    // Initialize rotated image row to black
    for (int x = 0; x < imageWidth; x++) {
      rotatedImage[i][x] = TRANSPARENT_COLOR;  // Used to avoid overwriting areas of background if untouched by rotated image, will not be written on display
    }
  }
  return true;
}

bool loadPngFromSD(const char *filename) {
  // Open the file
  File pngFile = SD.open(filename, FILE_READ);
  if (!pngFile) {
    Serial.println("Failed to open PNG file");
    return false;
  }

  // Create a buffer to hold the file data
  size_t fileSize = pngFile.size();
  uint8_t *pngBuffer = (uint8_t *)malloc(fileSize);
  if (!pngBuffer) {
    Serial.println("Not enough memory to load PNG");
    pngFile.close();
    return false;
  }

  // Read the entire file into the buffer
  pngFile.read(pngBuffer, fileSize);
  pngFile.close();

  // Decode the PNG from the buffer
  int16_t rc = png.openRAM(pngBuffer, fileSize, pngDraw);
  if (rc != PNG_SUCCESS) {
    Serial.printf("PNG decode error: %d\n", rc);
    free(pngBuffer);
    return false;
  }

  // Get the image dimensions
  imageWidth = png.getWidth();
  imageHeight = png.getHeight();

  // Check if image dimensions are too large
  if (imageWidth > MAX_IMAGE_WIDTH || imageHeight > MAX_IMAGE_HEIGHT) {
    Serial.println("Image too large for buffer");
    free(pngBuffer);
    return false;
  }

  // Allocate memory for image buffers
  if (!allocateImageBuffers()) {
    Serial.println("Failed to allocate memory for image buffers");
    free(pngBuffer);
    return false;
  }

  // Decode the PNG into our buffer
  rc = png.decode(NULL, 0);

  // Free the buffer
  free(pngBuffer);

  return (rc == PNG_SUCCESS);
}

uint16_t getPixel(int x, int y) {  // Get a pixel from the original image
  if (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight && originalImage[y] != NULL) {
    return originalImage[y][x];
  }
  return TFT_BLACK;  // Return black for out-of-bounds pixels
}

void setPixel(int x, int y, uint16_t color) {  // Set a pixel in the rotated image
  if (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight && rotatedImage[y] != NULL) {
    rotatedImage[y][x] = color;
  }
}

void rotateImage(float angle) {  // Function to rotate image by arbitrary angle
  // Convert angle to radians
  float rad = angle * PI / 180.0;
  float sinma = sin(rad);
  float cosma = cos(rad);

  // Find the center of the image
  float cx = imageWidth / 2.0;
  float cy = imageHeight / 2.0;

  // Perform rotation with nearest neighbor sampling for speed
  for (int y = 0; y < imageHeight; y++) {
    for (int x = 0; x < imageWidth; x++) {
      // Calculate the source coordinates
      float srcX = cosma * (x - cx) + sinma * (y - cy) + cx;
      float srcY = -sinma * (x - cx) + cosma * (y - cy) + cy;

      // Check if the source coordinates are within bounds
      if (srcX >= 0 && srcX < imageWidth && srcY >= 0 && srcY < imageHeight) {
        // Use nearest neighbor sampling for simplicity and speed
        int sx = (int)(srcX + 0.5);
        int sy = (int)(srcY + 0.5);

        // Copy the pixel
        setPixel(x, y, getPixel(sx, sy));
      }
    }
  }
}
