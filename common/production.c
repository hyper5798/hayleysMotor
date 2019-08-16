#include "production.h"
#include <string.h>
#include "utilities.h"

#define UartDataTypeUART            "UART"
#define UartDataTypeTXCM            "TXCM"
#define UartDataTypeTXPP            "TXPP"
#define UartDataTypeTXNM            "TXNM"
#define UartDataTypeQUIT            "QUIT"
#define UartDataTypeHELP            "HELP"
#define UartDataTypeMAC             "MAC"
#define UartDataTypeLED             "LED"
#define UartDataTypeSNUM            "SNUM"
#define UartDataTypeVER             "VER"
#define UartDataTypeRX              "RX"
#define UartDataTypeRSSI            "RSSI"
#define UartDataTypeLNA             "LNA"
#define UartDataTypeSX1276          "SX1276"
#define UartDataTypeEXIT            "EXIT"
#define UartDataTypeCONFIG          "CONF"
#define UartDataTypePROFILE         "PROF"
#define UartDataTypeMODID           "MODID"
#define UartDataTypeSYSID           "SYSID"
#define UartDataTypeNWKSKEY         "NSKEY"
#define UartDataTypeAPPSKEY         "ASKEY"
#define UartDataTypeGSENSOR         "GSEN"
#define UartDataTypeTEMPERATURE     "TEMP"
#define UartDataTypeICTEMPERATURE   "ICTEMP"
#define UartDataTypeGPS             "GPS"
#define UartDataTypeFACTORY         "FAC"
#define UartDataTypeCAP             "CAP"
#define UartDataTypeUPDATECONFIG    "UPCONF"
#define UartDataTypeBATCAL          "BATCAL"
#define UartDataTypeBATREAD         "BATR"
#define UartDataTypeBUZZ            "BUZZ"
#define UartDataTypeDEBUG           "DEBUG"
#define UartDataTypeFREQ            "FREQ"
#define UartDataTypeEXTEEPROM       "EXTEEPROM"
#define UartDataTypeLOWPOWER        "LOWP"
#define UartDataTypeHARDFAULTREAD   "HARDR"
#define UartDataTypeHARDFAULTWRITE  "HARDW"
#define UartDataTypeFWCOUNTRY       "FWCOUNTRY"
#define UartDataTypeLORARESET       "LORARST"
#define UartDataTypeAPPEUI          "APPEUI"
#define UartDataTypeDEVEUI          "DEVEUI"
#define UartDataTypeAPPKEY          "APPKEY"
#define UartDataTypeLORACFG         "LORACFG"
#define UartDataTypePROV            "PROV"
#define UartDataTypeCADINFO         "CADINFO"

NodeTest checkCmd(uint8_t *string)
{
    if( *(string) == '\r' || *(string) == '\n' )
        return PROD_TEST_ENTER;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeUART, strlen(UartDataTypeUART) ) == 0 )
        return PROD_TEST_UART_START;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeTXCM, strlen(UartDataTypeTXCM) ) == 0 )
        return PROD_TEST_TXCM_START;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeTXPP, strlen(UartDataTypeTXPP) ) == 0 )
        return PROD_TEST_TXPP_START;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeVER, strlen(UartDataTypeVER) ) == 0 )
        return PROD_TEST_VER;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeMAC, strlen(UartDataTypeMAC) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeMAC) + 1) == 'R')
            return PROD_TEST_MAC_READ;
        else if(*(string + strlen(UartDataTypeMAC) + 1) == 'W')
            return PROD_TEST_MAC_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeSNUM, strlen(UartDataTypeSNUM) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeSNUM) + 1) == 'R')
            return PROD_TEST_SNUM_READ;
        else if(*(string + strlen(UartDataTypeSNUM) + 1) == 'W')
            return PROD_TEST_SNUM_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeNWKSKEY, strlen(UartDataTypeNWKSKEY) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeNWKSKEY) + 1) == 'R')
            return PROD_TEST_NWKSKEY_READ;
        else if(*(string + strlen(UartDataTypeNWKSKEY) + 1) == 'W')
            return PROD_TEST_NWKSKEY_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeAPPSKEY, strlen(UartDataTypeAPPSKEY) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeAPPSKEY) + 1) == 'R')
            return PROD_TEST_APPSKEY_READ;
        else if(*(string + strlen(UartDataTypeAPPSKEY) + 1) == 'W')
            return PROD_TEST_APPSKEY_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeMODID, strlen(UartDataTypeMODID) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeMODID) + 1) == 'R')
            return PROD_TEST_MODID_READ;
        else if(*(string + strlen(UartDataTypeMODID) + 1) == 'W')
            return PROD_TEST_MODID_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeSYSID, strlen(UartDataTypeSYSID) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeSYSID) + 1) == 'R')
            return PROD_TEST_SYSID_READ;
        else if(*(string + strlen(UartDataTypeSYSID) + 1) == 'W')
            return PROD_TEST_SYSID_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeCONFIG, strlen(UartDataTypeCONFIG) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeCONFIG) + 1) == 'R')
            return PROD_TEST_CONFIG_READ;
        else if(*(string + strlen(UartDataTypeCONFIG) + 1) == 'W')
            return PROD_TEST_CONFIG_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypePROFILE, strlen(UartDataTypePROFILE) ) == 0 )
    {
        if(*(string + strlen(UartDataTypePROFILE) + 1) == 'R')
            return PROD_TEST_PROFILE_READ;
        else if(*(string + strlen(UartDataTypePROFILE) + 1) == 'W')
            return PROD_TEST_PROFILE_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeGSENSOR, strlen(UartDataTypeGSENSOR) ) == 0 )
    {
#ifdef TRACKER_BOARD
        return PROD_TEST_GSENSOR_READ;
#elif MANHOLE_BOARD
        return PROD_TEST_GSENSOR_READ;
#elif TREE_TRACKER_BOARD
        return PROD_TEST_GSENSOR_READ;
#else
        return PROD_TEST_GSENSOR_READ;
#endif
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeBATCAL, strlen(UartDataTypeBATCAL) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeBATCAL) + 1) == 'R')
            return PROD_TEST_BAT_CAL_READ;
        else if(*(string + strlen(UartDataTypeBATCAL) + 1) == 'C')
            return PROD_TEST_BAT_CAL_START;
        else if(*(string + strlen(UartDataTypeBATCAL) + 1) == 'W')
            return PROD_TEST_BAT_CAL_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeBATREAD, strlen(UartDataTypeBATREAD) ) == 0 )
        return PROD_TEST_BAT_READ;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeTEMPERATURE, strlen(UartDataTypeTEMPERATURE) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeTEMPERATURE) + 1) == '0')
            return PROD_TEST_TEMPERATURE_READ;
        else if(*(string + strlen(UartDataTypeTEMPERATURE) + 1) == '1')
            return PROD_TEST_TEMPERATURE_START;
        else if(*(string + strlen(UartDataTypeTEMPERATURE) + 1) == '2')
            return PROD_TEST_TEMPERATURE_CAL_READ;
        else if(*(string + strlen(UartDataTypeTEMPERATURE) + 1) == '3')
            return PROD_TEST_TEMPERATURE_CAL_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeLED, strlen(UartDataTypeLED) ) == 0 )
        return PROD_TEST_LED;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeGPS, strlen(UartDataTypeGPS) ) == 0 )
        return PROD_TEST_GPS_START;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeBUZZ, strlen(UartDataTypeBUZZ) ) == 0 )
        return PROD_TEST_BUZZ;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeQUIT, strlen(UartDataTypeQUIT) ) == 0 )
        return PROD_TEST_QUIT;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeEXIT, strlen(UartDataTypeEXIT) ) == 0 )
        return PROD_TEST_EXIT;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeRX, strlen(UartDataTypeRX) ) == 0 )
        return PROD_TEST_RX;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeICTEMPERATURE, strlen(UartDataTypeICTEMPERATURE) ) == 0 )
        return PROD_TEST_ICTEMPERATURE;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeTXNM, strlen(UartDataTypeTXNM) ) == 0 )
        return PROD_TEST_TXNM_START;
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeDEBUG, strlen(UartDataTypeDEBUG) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeDEBUG) + 1) == 'R')
            return PROD_TEST_DEBUG_READ;
        else if(*(string + strlen(UartDataTypeDEBUG) + 1) == 'W')
            return PROD_TEST_DEBUG_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeFREQ, strlen(UartDataTypeFREQ) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeFREQ) + 1) == 'R')
            return PROD_TEST_FREQ_READ;
        else if(*(string + strlen(UartDataTypeFREQ) + 1) == 'W')
            return PROD_TEST_FREQ_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeEXTEEPROM, strlen(UartDataTypeEXTEEPROM) ) == 0 )
    {
        return PROD_TEST_EXT_EEPROM;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeLOWPOWER, strlen(UartDataTypeLOWPOWER) ) == 0 )
    {
        return PROD_TEST_LOW_POWER;
    }
        else if( strncmp( ( const char* )string, ( const char* )UartDataTypeHARDFAULTREAD, strlen(UartDataTypeHARDFAULTREAD) ) == 0 )
    {
        return PROD_TEST_HARDFAULT_READ;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeHARDFAULTWRITE, strlen(UartDataTypeHARDFAULTWRITE) ) == 0 )
    {
        return PROD_TEST_HARDFAULT_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeFWCOUNTRY, strlen(UartDataTypeFWCOUNTRY) ) == 0 )
        return PROD_TEST_FW_COUNTRY;
    
    /* Modified by Crux, Date: 2017/04/12, Log: Add command to reset LoRa config in MFT mode. */
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeLORARESET, strlen(UartDataTypeLORARESET) ) == 0 )
        return PROD_TEST_LORA_RESET;
    
    /* Modified by Crux, Date: 2017/04/19, Log: Add command to set APP EUI and DEV EUI in MFT mode. */
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeAPPEUI, strlen(UartDataTypeAPPEUI) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeAPPEUI) + 1) == 'R')
            return PROD_TEST_APP_EUI_READ;
        else if(*(string + strlen(UartDataTypeAPPEUI) + 1) == 'W')
            return PROD_TEST_APP_EUI_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeDEVEUI, strlen(UartDataTypeDEVEUI) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeDEVEUI) + 1) == 'R')
            return PROD_TEST_DEV_EUI_READ;
        else if(*(string + strlen(UartDataTypeDEVEUI) + 1) == 'W')
            return PROD_TEST_DEV_EUI_WRITE;
    }
    else if ( strncmp( ( const char* )string, ( const char* )UartDataTypeLORACFG, strlen(UartDataTypeLORACFG) ) == 0 ) {
        return PROD_TEST_LORA_CONFIGURATION;
    }
    /* Modified by Crux, Date: 2017/06/30, Log: Add command to provision. */
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypePROV, strlen(UartDataTypePROV) ) == 0 )
    {
        if(*(string + strlen(UartDataTypePROV) + 1) == 'R')
            return PROD_TEST_PROV_READ;
        else if(*(string + strlen(UartDataTypePROV) + 1) == 'W')
            return PROD_TEST_PROV_WRITE;
    }
    /* Modified by Crux, Date: 2017/07/24, Log: Add command to read/write APP key. */
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeAPPKEY, strlen(UartDataTypeAPPKEY) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeAPPKEY) + 1) == 'R')
            return PROD_TEST_APP_KEY_READ;
        else if(*(string + strlen(UartDataTypeAPPKEY) + 1) == 'W')
            return PROD_TEST_APP_KEY_WRITE;
    }
    else if( strncmp( ( const char* )string, ( const char* )UartDataTypeCADINFO, strlen(UartDataTypeCADINFO) ) == 0 )
    {
        if(*(string + strlen(UartDataTypeCADINFO) + 1) == 'R')
            return PROD_TEST_CAD_INFO_READ;
        else if(*(string + strlen(UartDataTypeCADINFO) + 1) == 'W')
            return PROD_TEST_CAD_INFO_WRITE;
    }

    return PROD_TEST_FAIL;
}

NodeTest parseTXCMdata(int8_t *string, txcm_t *data)
{
    uint8_t fieldSize = 0;
    uint16_t i = 0;
    uint8_t j = 0;
    if( strncmp( ( const char* )string, ( const char* )UartDataTypeTXCM, 4 ) != 0 )
        return PROD_TEST_FAIL;
    
    RemoveEndChar((uint8_t*)string);
    
    while( string[i + fieldSize++] != ',' )
    {
        if(fieldSize > 5)/* cmd len*/
            return PROD_TEST_FAIL;
    }
    i = 5;
    
    /*parse frequency*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if(fieldSize > 8)/* frequency len*/
            return PROD_TEST_FAIL;
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        data->LoRaFreq[j] = string[i];
    }
    data->LoRaFreq[j-1] = '\0';
    
    /*parse power*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if(fieldSize > 3)/* power len*/
            return PROD_TEST_FAIL;
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        data->LoRaTxPwr[j] = string[i];
    }
    data->LoRaTxPwr[j-1] = '\0';
    
    /*parse LoRaBwd*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 2 )/*Bwd len*/
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        data->LoRaBwd[j] = string[i];
    }
    data->LoRaBwd[j-1] = '\0';
    
    /* parse LoRaSpFactor*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 2 )/*SF len*/
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        data->LoRaSpFactor[j] = string[i];
    }
    data->LoRaSpFactor[j-1] = '\0';
    
    /* parse LoRaCodeRate*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 2 )/*CodeRate len*/
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        data->LoRaCodeRate[j] = string[i];
    }
    data->LoRaCodeRate[j-1] = '\0';
    
    /* parse LoRaFixLenPayloadOn*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 2 )/*PayloadOn len*/
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        data->LoRaFixLenPayloadOn[j] = string[i];
    }
    data->LoRaFixLenPayloadOn[j-1] = '\0';
    
    /* parse LoRaEnableCrc*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 2 )/*Enable crc len*/
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        data->LoRaEnableCrc[j] = string[i];
    }
    data->LoRaEnableCrc[j-1] = '\0';
    
    /* parse LoRaIqInversionOn*/
    fieldSize = 0;
    while( string[i + fieldSize++] != '\0' )
    {
        if( fieldSize > 2 )/*IQ len*/
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        data->LoRaIqInversionOn[j] = string[i];
    }
    data->LoRaIqInversionOn[j-1] = '\0';
    
    return PROD_TEST_TXCM_START;
}

NodeTest parseTXPPdata(int8_t *string, txpp_t *data, txcm_t *txcmdata)
{
    uint8_t fieldSize = 0;
    uint16_t i = 0;
    uint8_t j = 0;
    if( strncmp( ( const char* )string, ( const char* )UartDataTypeTXPP, 4 ) != 0 )
        return PROD_TEST_FAIL;
    
    RemoveEndChar((uint8_t*)string);
    
    while( string[i + fieldSize++] != ',' )
    {        
        if(fieldSize > 5)/* cmd len*/
            return PROD_TEST_FAIL;
    }
    i = 5;
    
    /*parse master/slave*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 2 )
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        data->PingPongMaster[j] = string[i];
    }
    data->PingPongMaster[j-1] = '\0';
    
    /*MaxTxNumOrRxTime, total times and total time interval*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 5 )
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        data->PingPongMaxTxNumOrRxTime[j] = string[i];
    }
    data->PingPongMaxTxNumOrRxTime[j-1] = '\0';
    
    /*parse frequency*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if(fieldSize > 8)/* frequency len*/
            return PROD_TEST_FAIL;
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        txcmdata->LoRaFreq[j] = string[i];
    }
    txcmdata->LoRaFreq[j-1] = '\0';
    
    /*parse power*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if(fieldSize > 3)/* power len*/
            return PROD_TEST_FAIL;
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        txcmdata->LoRaTxPwr[j] = string[i];
    }
    txcmdata->LoRaTxPwr[j-1] = '\0';
    
    /*parse LoRaBwd*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 2 )/*Bwd len*/
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        txcmdata->LoRaBwd[j] = string[i];
    }
    txcmdata->LoRaBwd[j-1] = '\0';
    
    /* parse LoRaSpFactor*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 2 )/*SF len*/
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        txcmdata->LoRaSpFactor[j] = string[i];
    }
    txcmdata->LoRaSpFactor[j-1] = '\0';
    
    /* parse LoRaCodeRate*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 2 )/*CodeRate len*/
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        txcmdata->LoRaCodeRate[j] = string[i];
    }
    txcmdata->LoRaCodeRate[j-1] = '\0';
    
    /* parse LoRaFixLenPayloadOn*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 2 )/*PayloadOn len*/
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        txcmdata->LoRaFixLenPayloadOn[j] = string[i];
    }
    txcmdata->LoRaFixLenPayloadOn[j-1] = '\0';
    
    /* parse LoRaEnableCrc*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 2 )/*Enable crc len*/
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        txcmdata->LoRaEnableCrc[j] = string[i];
    }
    txcmdata->LoRaEnableCrc[j-1] = '\0';
    
    /* parse LoRaIqInversionOn*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 2 )/*IQ len*/
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        txcmdata->LoRaIqInversionOn[j] = string[i];
    }
    txcmdata->LoRaIqInversionOn[j-1] = '\0';
    
    /*Timeout value for Tx or Rx*/
    fieldSize = 0;
    while( string[i + fieldSize++] != ',' )
    {
        if( fieldSize > 5 )
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        data->PingPongTxOrRxTimeout[j] = string[i];
    }
    data->PingPongTxOrRxTimeout[j-1] = '\0';
    
    /*Label value for Tx or Rx*/
    fieldSize = 0;
    while( string[i + fieldSize++] != '\0' )
    {
        if( fieldSize > 5 )
        {
            return PROD_TEST_FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        data->Label[j] = string[i];
    }
    data->Label[j-1] = '\0';
    
    return PROD_TEST_TXPP_START;
}



/************************************************************
* @brief        check if LoRa MAC is valid
*
* @param[in]    mac_str     : LoRa MAC for checking
*
* @return       PROD_TEST_SUCCESS   : LoRa MAC is valid
*               PROD_TEST_FAIL      : LoRa MAC is illegal
************************************************************/
NodeTest isLoRaMacValid(uint8_t *mac_str)
{
#if defined (TRACKER_BOARD) || defined (SIPMODULE_BOARD ) || defined (TRACKER_BOARD_V1)
    uint8_t i, mac_len = 8;
    if(strlen((char*)mac_str) != mac_len) //check MAC length
        return PROD_TEST_FAIL;
//    if( *(mac_str+2) != ':' || *(mac_str+5) != ':' || *(mac_str+8) != ':' ) //check ':'
//        return PROD_TEST_FAIL;
    
    for(i=0; i<mac_len; i++)
    {
//        if(i==2 || i==5 || i==8)
//            continue;
        if(*(mac_str+i) < '0' || *(mac_str+i) > '9')
        {
            if(*(mac_str+i) < 'a' || *(mac_str+i) > 'f')
            {
                if(*(mac_str+i) < 'A' || *(mac_str+i) > 'F')
                    return PROD_TEST_FAIL;
            }
        }
    }
    return PROD_TEST_SUCCESS;
#elif defined(MANHOLE_BOARD) || defined(EARTAG_BOARD)
    uint8_t i, mac_len = 8;
    if(strlen((char*)mac_str) != mac_len) //check MAC length
        return PROD_TEST_FAIL;
//    if( *(mac_str+2) != ':' || *(mac_str+5) != ':' || *(mac_str+8) != ':' ) //check ':'
//        return PROD_TEST_FAIL;
    
    for(i=0; i<mac_len; i++)
    {
//        if(i==2 || i==5 || i==8)
//            continue;
        if(*(mac_str+i) < '0' || *(mac_str+i) > '9')
        {
            if(*(mac_str+i) < 'a' || *(mac_str+i) > 'f')
            {
                if(*(mac_str+i) < 'A' || *(mac_str+i) > 'F')
                    return PROD_TEST_FAIL;
            }
        }
    }
    return PROD_TEST_SUCCESS;
#elif TREE_TRACKER_BOARD
    uint8_t i, mac_len = 8;
    if(strlen((char*)mac_str) != mac_len) //check MAC length
        return PROD_TEST_FAIL;
//    if( *(mac_str+2) != ':' || *(mac_str+5) != ':' || *(mac_str+8) != ':' ) //check ':'
//        return PROD_TEST_FAIL;

    for(i=0; i<mac_len; i++)
    {
//        if(i==2 || i==5 || i==8)
//            continue;
        if(*(mac_str+i) < '0' || *(mac_str+i) > '9')
        {
            if(*(mac_str+i) < 'a' || *(mac_str+i) > 'f')
            {
                if(*(mac_str+i) < 'A' || *(mac_str+i) > 'F')
                    return PROD_TEST_FAIL;
            }
        }
    }
    return PROD_TEST_SUCCESS;
#else
    return PROD_TEST_FAIL;
#endif
}

/************************************************************
* @brief        check if SN is valid
*
* @param[in]    sn_str      : SN for checking
*
* @return       PROD_TEST_SUCCESS   : SN is valid
*               PROD_TEST_FAIL      : SN is illegal
************************************************************/
NodeTest isSnValid(uint8_t *sn_str)
{
#if defined (TRACKER_BOARD) || defined (SIPMODULE_BOARD ) || defined (TRACKER_BOARD_V1)
    if(strlen((char*)sn_str) == 13) //check length
        return PROD_TEST_SUCCESS;
    return PROD_TEST_FAIL;
#elif MANHOLE_BOARD 
    if(strlen((char*)sn_str) == 13) //check length
        return PROD_TEST_SUCCESS;
    return PROD_TEST_FAIL;
#elif TREE_TRACKER_BOARD
    if(strlen((char*)sn_str) == 13) //check length
        return PROD_TEST_SUCCESS;
    return PROD_TEST_FAIL;
#elif EARTAG_BOARD 
    if(strlen((char*)sn_str) == 13) //check length
        return PROD_TEST_SUCCESS;
    return PROD_TEST_FAIL;
#else
    return PROD_TEST_FAIL;
#endif
}

/************************************************************
* @brief        check if key is valid
*
* @param[in]    key_str     : key for checking
*
* @return       PROD_TEST_SUCCESS   : key is valid
*               PROD_TEST_FAIL      : key is illegal
************************************************************/
NodeTest isAesKeyValid(uint8_t *key_str)
{
#if defined (TRACKER_BOARD) || defined (SIPMODULE_BOARD ) || defined (TRACKER_BOARD_V1) || defined(MANHOLE_BOARD) || defined(EARTAG_BOARD)
    if(strlen((char*)key_str) == 32) //check key length
        return PROD_TEST_SUCCESS;
    return PROD_TEST_FAIL;
#elif TREE_TRACKER_BOARD
    if(strlen((char*)key_str) == 32) //check key length
        return PROD_TEST_SUCCESS;
    return PROD_TEST_FAIL;
#else
    return PROD_TEST_FAIL;
#endif
    
}

/*!
 * MFT command parser.
 * Author: Crux
 *
 * example:
 *
 * uint8_t cmd[] = "NSKEY,W,aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
 * NodeTestCmdParseBuff_t parse_buff;
 * uint8_t ret;
 * 
 * ret = parseTestCmd(cmd, &parse_buff);
 * 
 * //result:
 * // cmd = NSKEY,W,aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
 * // parse_buff.argc = 3
 * // (parse_buff.cmd + parse_buff.argv_ptr[0]): NSKEY
 * // (parse_buff.cmd + parse_buff.argv_ptr[1]): W
 * // (parse_buff.cmd + parse_buff.argv_ptr[2]): aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
 *
 * \param [IN] cmd      input command
 * \param [OUT] buff    parse result
 * \param [return]      PROD_TEST_FAIL or PROD_TEST_SUCCESS
 */
NodeTest parseTestCmd(uint8_t *cmd, NodeTestCmdParseBuff_t *buff)
{
    uint8_t i, len;
    
    if(cmd == NULL || buff == NULL)
        return PROD_TEST_FAIL;
    
    len = strlen((char*)cmd);
    if(len == 0)
        return PROD_TEST_FAIL;
    
    strcpy((char*)buff->cmd, (char*)cmd);
    buff->argc = 1;
    buff->argv_ptr[0] = 0;
    
    for(i=0; i<len; i++)
    {
        if(*(buff->cmd + i) == ',')
        {
            *(buff->cmd + i) = '\0';
            buff->argv_ptr[buff->argc] = i+1;
            buff->argc++;
        }
    }
    return PROD_TEST_SUCCESS;
}

