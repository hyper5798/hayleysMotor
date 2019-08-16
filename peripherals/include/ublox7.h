#ifndef __UBLOX7_H__
#define __UBLOX7_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>
    
typedef void( GpsIrqHandler )(void);

void Ublox7SetPpsCallback(GpsIrqHandler *irqHandler);
void Ublox7OnPpsSignal( void );
double Ublox7ConverLatitudeFromDMMtoDD(char *NmeaLatitude);
double Ublox7ConverLongitudeFromDMMtoDD(char *NmeaLongitude);
bool Ublox7ConverLatitudePole(char *NmeaLatitudePole);
bool Ublox7ConverLongitudePole(char *NmeaLongitudePole);

#ifdef __cplusplus
}
#endif
#endif // __UBLOX7_H__
