Moon Phase Display Based on Observer's Location  

This project visualizes moon phases as they appear in the sky, adjusting for the observer's position on Earth.  

Features  
- Displays 40 moon phase images (at 9-degree increments) with realistic progression over time.  
- Takes the observer’s location into account to tilt the illuminated limb correctly.  
- Uses formulas from Jean Meeus' Astronomical Algorithms (1998).  
- Includes Blue Moon date calculations — added for fun since some of the necessary data was already in use for moon phase calculations.  

Moon Phase Calculation  
- The moon's average period is 29.53 days, but it varies by ±15 hours, requiring precise calculations for Blue Moons, equinoxes, and solstices.  
- Meeus' formula defines the illumination angle starting from the top of the moon and increasing counterclockwise (CCW). For easier readability, this implementation inverts the angle to increase clockwise (CW).  
- When the moon is below the horizon (Alt < 0), the display remains hypothetical and intentionally ignores Earth's obstruction.  

Hardware Setup  
- Designed for ESP32-Wroom, though any ESP32 dev board should work.  
- Uses two ILI9341 TFT displays sharing a single SPI bus.  
- Moon phase images are stored on an SD card using a separate SPI bus, inserted into one of the TFT display slots.  

Controls & Settings  
- Rotary encoder adjusts Time, Date, Timezone, and Observer Latitude/Longitude.  
- Settings Mode:  
  - Long-press to enter.  
  - A box will appear at the first setting (Month).  
  - Turn to adjust values, short-click to advance to the next setting.  
- Timezone & Location:  
  - Timezone and coordinates are stored in RTC NVRAM.  
  - A few predefined timezones (USA, Britain, Western Europe) are included—modify the TZ-Data array as needed.  
  - RTC operates on UTC, with displayed time converted to local time.  
  - Get the current latitude/longitude using Google (e.g., `"lat lon minneapolis"`), rounding to the nearest full degree.  
  - Use positive values for North (N) and East (E), negative values*for South (S) and West (W).  
  - Example: Minneapolis, USA - 44.9778° N, 93.2650° W → Set Lat = 45, Lon = -93.  

Battery Modification  
- The RTC module's original LIR2032 battery holder was removed due to a design flaw causing continuous overcharging.  
- Replaced with a 1.5F supercapacitor for better performance.  

All PIN connections and their respective functions are listed in the remarks within the sketch file.  
