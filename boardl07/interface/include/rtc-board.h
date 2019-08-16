#ifndef __RTC_BOARD_H__
#define __RTC_BOARD_H__

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>
#include <stm32l0xx.h>

// Unix epoch time in Julian calendar (UnixTime = 00:00:00 01.01.1970 => JDN = 2440588)
#define JULIAN_DATE_BASE    2440588
#define SECS_IN_MIN         ( ( uint8_t ) 60 )
#define SECS_IN_HR          ( ( uint16_t ) 3600 )
#define SECS_IN_DAY         ( ( uint32_t ) 86400 )
#define HRS_IN_DAY          ( ( uint8_t ) 24 )

typedef enum
{
    MCU_ALARM_A = 0x0,
    MCU_ALARM_B
}MCUALARM_e;

typedef void( RtcAlarmIrqHandler )( MCUALARM_e AlarmName );
bool RtcInit( void );
void RtcSetAlarmCallback( RtcAlarmIrqHandler *irqHandler );
bool RtcSetConfig( void );
static void RtcSetAlarmConfig( void );
void RtcGetTime(uint8_t *h, uint8_t *m, uint8_t *s);
void RtcTimeCalibration(uint8_t *h, uint8_t *m, uint8_t *s);
bool RtcSetTime(uint8_t h, uint8_t m, uint8_t s);
void RtcGetDate(uint8_t *y, uint8_t *m, uint8_t *d);
bool RtcSetDate(uint8_t y, uint8_t m, uint8_t d);
void RtcSetAlarm(uint8_t h, uint8_t m, uint8_t s, MCUALARM_e AlarmName);
void RtcSetDateAlarm(uint8_t d, uint8_t h, uint8_t m, uint8_t s, MCUALARM_e AlarmName);
void RtcClearStatus( MCUALARM_e AlarmName );
void RtcDisableAlarm( MCUALARM_e AlarmName );

/*!
 * Enable RTC Alarm with Epoch Time
 *
 * \param [IN] timeoutValue     A 32-bit value in Epoch Time Seconds
 *
 *     usage: Starts an alarm 20 secs from now on
 *
 *     RtcStartWakeUpAlarm( 20 );
 */
void RtcStartWakeUpAlarm( uint32_t timeoutValue, MCUALARM_e AlarmName);
uint32_t RtcGetEpochFromUTC( uint8_t YY, uint8_t MM, uint8_t DD, uint8_t hh, uint8_t mm, uint8_t ss );
void RtcGetUTCFromEpoch( uint32_t epoch, uint8_t *YY, uint8_t *MM, uint8_t *DD, uint8_t *hh, uint8_t *mm, uint8_t *ss );
uint32_t RtcGetEpochTimeStamp(void);

#ifdef __cplusplus
}
#endif
#endif /*__RTC_BOARD_H__*/
