#ifndef __HWTIMER_BOARD_H__
#define __HWTIMER_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32l0xx.h"
#include <stdbool.h>

#ifdef LORAWANV1_0
#ifndef TimerTime_t
typedef uint64_t TimerTime_t;
#endif
#endif

typedef void( HwTimerIrqHandler )( void );
void HwTimerSetCallback(HwTimerIrqHandler * irqHandler);
void HwTimerInit(void);
void HwTimerDeInit(void);
void HwTimerStart(void);
void HwTimerStop(void);
bool HwTimerBusy(void);

#ifdef LORAWANV1_0
TimerTime_t TimerHwGetTime( void );

#ifdef TIMER_BASE_MS
/*!
 * \brief Return the Time elapsed since a fix moment in Time
 *
 * \param [IN] savedTime    fix moment in Time
 * \retval time             returns elapsed time
 */

TimerTime_t TimerHwGetElapsedTime( TimerTime_t savedTime );

void HwTimerBackupTickCounter(void);
void HwTimerResume(void);

#endif
#endif

#ifdef __cplusplus
}
#endif
#endif /*__HWTIMER_BOARD_H__*/
