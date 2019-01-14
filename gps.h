#ifndef __GPS_H__
#define __GPS_H__

#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define GPS_TX 12
#define GPS_RX 15

class gps
{
    public:
        void init();
        bool checkGpsFix();
        void getLatLon(double* lat, double* lon, double *alt, double *kmph, int *sats);
        void encode();

    private:
        char t[32]; // used to sprintf for Serial output
        TinyGPSPlus tGps;
};

#endif
