#ifndef __RADIOCHANNEL_H__
#define __RADIOCHANNEL_H__
#ifdef __cplusplus
 extern "C" {
#endif
/*!
 * LoRaMac maximum number of channels
 */
#include "LoRaMac.h"
#ifdef USE_16CH
#define LORA_MAX_NB_CHANNELS                        16
#else
#define LORA_MAX_NB_CHANNELS                        8
#endif

#define RxDelayTime 1000

/*!
 * Second reception window channel definition.
 * \remark DCycle field isn't used. This channel is Rx only
 */
#ifdef SIPMODULE_G78S
#define MODULE_A_PLL_1											470600000 //486000000
#define MODULE_A_PLL_2											471400000 //486800000
#define MODULE_B_PLL_1											472200000 //487600000
#define MODULE_B_PLL_2											473000000 //488400000
#else
#define MODULE_A_PLL_1											920500000 //923000000
#define MODULE_A_PLL_2											921300000 //924000000
#define MODULE_B_PLL_1											922100000 //925500000
#define MODULE_B_PLL_2											923500000 //926500000
#endif

#define CONFIRMED_MSG_BW										LORA_BW_125
#define UNCONFIRMED_MSG_BW									LORA_BW_500

#define TX_SF_MAX														DR_SF7H
#define TX_SF_MIN														DR_SF12

#define RX_WND_2_CHANNEL                                  { ( MODULE_B_PLL_2 ), DR_SF10 }		//hanson mod
/*!
 * Default Tx output power used by the node
 */
#define LORAMAC_DEFAULT_TX_POWER                    TX_POWER_20_DBM

/*!
 * Default datarate used by the node
 */
#define LORAMAC_DEFAULT_DATARATE                    DR_SF10

/*!
 * LoRaMac default channels
 */
#if USE_8CH_MODULE_A
#define LC1                { ( MODULE_A_PLL_1 - 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC2                { ( MODULE_A_PLL_1 - 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC3                { ( MODULE_A_PLL_1 + 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC4                { ( MODULE_A_PLL_1 + 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC5                { ( MODULE_A_PLL_2 - 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC6                { ( MODULE_A_PLL_2 - 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC7                { ( MODULE_A_PLL_2 + 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC8                { ( MODULE_A_PLL_2 + 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }

#elif USE_8CH_MODULE_B
#define LC1                { ( MODULE_B_PLL_1 - 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC2                { ( MODULE_B_PLL_1 - 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC3                { ( MODULE_B_PLL_1	+ 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC4                { ( MODULE_B_PLL_1 + 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC5                { ( MODULE_B_PLL_2 - 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC6                { ( MODULE_B_PLL_2 - 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC7                { ( MODULE_B_PLL_2 + 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC8                { ( MODULE_B_PLL_2	+ 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }

#elif USE_16CH
#define LC1                { ( MODULE_A_PLL_1 - 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC2                { ( MODULE_A_PLL_1 - 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC3                { ( MODULE_A_PLL_1	+ 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC4                { ( MODULE_A_PLL_1 + 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC5                { ( MODULE_A_PLL_2 - 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC6                { ( MODULE_A_PLL_2 - 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC7                { ( MODULE_A_PLL_2 + 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC8                { ( MODULE_A_PLL_2	+ 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC9                { ( MODULE_B_PLL_1 - 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC10               { ( MODULE_B_PLL_1 - 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC11               { ( MODULE_B_PLL_1	+ 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC12               { ( MODULE_B_PLL_1 + 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC13               { ( MODULE_B_PLL_2 - 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC14               { ( MODULE_B_PLL_2 - 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC15               { ( MODULE_B_PLL_2 + 125000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#define LC16               { ( MODULE_B_PLL_2	+ 375000 ), { ( ( TX_SF_MAX << 4 ) | TX_SF_MIN ) }, 0 }
#else
		#error "Please define the number of channels in the compiler options."
#endif

static ChannelParams_t Channels[LORA_MAX_NB_CHANNELS] =
{
    LC1,
    LC2,
    LC3,
	LC4,
	LC5,
	LC6,
	LC7,
	LC8,
#ifdef USE_16CH
	LC9,
	LC10,
	LC11,
	LC12,
	LC13,
	LC14,
	LC15,
	LC16
#endif
};

static uint16_t ChannelsMask = LC( 1 )  + LC( 2 )  + LC( 3 )
+ LC( 4 )  + LC( 5 )  + LC( 6 )  + LC( 7 )  + LC( 8 )
#ifdef USE_16CH
+ LC( 9 )  + LC( 10 ) + LC( 11 ) + LC( 12 )
+ LC( 13 ) + LC( 14 ) + LC( 15 ) + LC( 16 )
#endif
;
#ifdef __cplusplus
}
#endif
#endif /*__RADIOCHANNEL_H__*/
