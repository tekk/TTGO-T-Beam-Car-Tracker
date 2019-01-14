#include "gps.h"

HardwareSerial GPSSerial(1);

void gps::init()
{
  GPSSerial.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
  GPSSerial.setTimeout(2);
}

void gps::encode()
{       
    int data;
    int previousMillis = millis();

    while((previousMillis + 1000) > millis())
    {
        while (GPSSerial.available() )
        {
            char data = GPSSerial.read();
            tGps.encode(data);
        }
    }
}

void gps::getLatLon(double* lat, double* lon, double *alt, double *kmph, int *sats)
{
  sprintf(t, "Lat: %f", tGps.location.lat());
  Serial.println(t);
  
  sprintf(t, "Lng: %f", tGps.location.lng());
  Serial.println(t);
  
  sprintf(t, "Alt: %f meters", tGps.altitude.meters());
  Serial.println(t);

  sprintf(t, "Speed: %f km/h", tGps.speed.kmph());
  Serial.println(t);

  sprintf(t, "Sats: %d", tGps.satellites.value());
  Serial.println(t);

  *lat = tGps.location.lat();
  *lon = tGps.location.lng();
  *alt = tGps.altitude.meters();
  *kmph = tGps.speed.kmph();
  *sats = tGps.satellites.value();
}

bool gps::checkGpsFix()
{
  encode();
  if (tGps.location.isValid() && 
      tGps.location.age() < 2000 &&
      tGps.hdop.isValid() &&
      tGps.hdop.value() <= 300 &&
      tGps.hdop.age() < 2000 &&
      tGps.altitude.isValid() && 
      tGps.altitude.age() < 2000 )
  {
    Serial.println("Valid gps Fix.");
    return true;
  }
  else
  {
    Serial.println("No gps Fix.");
    return false;
  }
}
