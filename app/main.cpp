/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "mbed.h"
#include "board.h"
#include "radio.h"

#include "LoRaMac.h"

#include "SerialDisplay.h"

/*!
 * When set to 1 the application uses the Over-the-Air activation procedure
 * When set to 0 the application uses the Personalization activation procedure
 */
#define OVER_THE_AIR_ACTIVATION                     1

/*!
 * Indicates if the end-device is to be connected to a private or public network
 */
#define LORAWAN_PUBLIC_NETWORK                      true

#if( OVER_THE_AIR_ACTIVATION != 0 )

/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           10000000  // 10 [s] value in us

/*!
 * Mote device IEEE EUI
 *
 * \remark must be written as a little endian value (reverse order of normal reading)
 */
#define LORAWAN_DEVICE_EUI                          { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF }

/*!
 * Application IEEE EUI
 *
 * \remark must be written as a little endian value (reverse order of normal reading)
 */
#define LORAWAN_APPLICATION_EUI                     { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

/*!
 * AES encryption/decryption cipher application key
 */
#define LORAWAN_APPLICATION_KEY                     { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x01 }

#else

/*!
 * Current network ID
 */
#define LORAWAN_NETWORK_ID                          ( uint32_t )0

/*!
 * Device address on the network
 *
 * \remark must be written as a big endian value (normal reading order)
 */
#define LORAWAN_DEVICE_ADDRESS                      ( uint32_t )0xFF000001

/*!
 * AES encryption/decryption cipher network session key
 */
#define LORAWAN_NWKSKEY                             { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x01 }

/*!
 * AES encryption/decryption cipher application session key
 */
#define LORAWAN_APPSKEY                             { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x01 }

#endif

/*!
 * Defines the application data transmission duty cycle. 5s, value in [us].
 */
#define APP_TX_DUTYCYCLE                            5000000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [us].
 */
#define APP_TX_DUTYCYCLE_RND                        1000000

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    true

/*!
 * LoRaWAN Adaptative Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

#if defined( USE_BAND_868 )

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        false

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            15

/*!
 * User application data buffer size
 */
#if ( LORAWAN_CONFIRMED_MSG_ON == 1 )
#define LORAWAN_APP_DATA_SIZE                       6

#else
#define LORAWAN_APP_DATA_SIZE                       1

#endif

#if( OVER_THE_AIR_ACTIVATION != 0 )

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#else

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

#endif

/*!
 * Indicates if the MAC layer has already joined a network.
 */
static bool IsNetworkJoined = false;
static bool IsNetworkJoinedStatusUpdate = false;

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

#if( OVER_THE_AIR_ACTIVATION != 0 )

/*!
 * Defines the join request timer
 */
static TimerEvent_t JoinReqTimer;

#endif

/*!
 * Indicates if a new packet can be sent
 */
static bool TxNextPacket = true;
static bool ScheduleNextTx = false;

static LoRaMacCallbacks_t LoRaMacCallbacks;

static TimerEvent_t Led1Timer;
volatile bool Led1State = false;
volatile bool Led1StateChanged = false;

static TimerEvent_t Led2Timer;
volatile bool Led2State = false;
volatile bool Led2StateChanged = false;

static bool AppLedStateOn = false;
volatile bool Led3StateChanged = false;

volatile bool LinkStatusUpdated = false;

static bool ComplianceTestOn = false;
static uint8_t ComplianceTestState = 0;
static uint16_t ComplianceTestDownLinkCounter = 0;
static bool ComplianceTestLinkCheck = false;
static uint8_t ComplianceTestDemodMargin = 0;
static uint8_t ComplianceTestNbGateways = 0;

struct sLoRaMacUplinkStatus
{
    uint8_t Acked;
    int8_t Datarate;
    uint16_t UplinkCounter;
    uint8_t Port;
    uint8_t *Buffer;
    uint8_t BufferSize;
}LoRaMacUplinkStatus;

struct sLoRaMacDownlinkStatus
{
    int16_t Rssi;
    int8_t Snr;
    uint16_t DownlinkCounter;
    bool RxData;
    uint8_t Port;
    uint8_t *Buffer;
    uint8_t BufferSize;
}LoRaMacDownlinkStatus;

void SerialDisplayRefresh( void )
{
    SerialDisplayInit( );
    SerialDisplayUpdateActivationMode( OVER_THE_AIR_ACTIVATION );

#if( OVER_THE_AIR_ACTIVATION == 0 )
    SerialDisplayUpdateNwkId( LORAWAN_NETWORK_ID );
    SerialDisplayUpdateDevAddr( LORAWAN_DEVICE_ADDRESS );
    SerialDisplayUpdateKey( 12, NwkSKey );
    SerialDisplayUpdateKey( 13, AppSKey );
#else
    SerialDisplayUpdateEui( 5, DevEui );
    SerialDisplayUpdateEui( 6, AppEui );
    SerialDisplayUpdateKey( 7, AppKey );
#endif
    SerialDisplayUpdateNetworkIsJoined( IsNetworkJoined );

    SerialDisplayUpdateAdr( LORAWAN_ADR_ON );
#if defined( USE_BAND_868 )
    SerialDisplayUpdateDutyCycle( LORAWAN_DUTYCYCLE_ON );
#else
    SerialDisplayUpdateDutyCycle( false );
#endif
    SerialDisplayUpdatePublicNetwork( LORAWAN_PUBLIC_NETWORK );
    
    SerialDisplayUpdateLedState( 3, AppLedStateOn );
}

void SerialRxProcess( void )
{
    if( SerialDisplayReadable( ) == true )
    {
        switch( SerialDisplayGetChar( ) )
        {
            case 'R':
            case 'r':
                // Refresh Serial screen
                SerialDisplayRefresh( );
                break;
            default:
                break;
        }
    }
}

/*!
 * Prepares the frame buffer to be sent
 */
static void PrepareTxFrame( uint8_t port )
{
    switch( port )
    {
    case 15:
        {
            AppData[0] = AppLedStateOn;
            if( IsTxConfirmed == true )
            {
                AppData[1] = LoRaMacDownlinkStatus.DownlinkCounter >> 8;
                AppData[2] = LoRaMacDownlinkStatus.DownlinkCounter;
                AppData[3] = LoRaMacDownlinkStatus.Rssi >> 8;
                AppData[4] = LoRaMacDownlinkStatus.Rssi;
                AppData[5] = LoRaMacDownlinkStatus.Snr;
            }
        }
        break;
    case 224:
        if( ComplianceTestLinkCheck == true )
        {
            ComplianceTestLinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTestDemodMargin;
            AppData[2] = ComplianceTestNbGateways;
            ComplianceTestState = 1;
        }
        else
        {
            switch( ComplianceTestState )
            {
            case 4:
                ComplianceTestState = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTestDownLinkCounter >> 8;
                AppData[1] = ComplianceTestDownLinkCounter;
                break;
            }
        }
        break;
    }
}

static void ProcessRxFrame( LoRaMacEventFlags_t *flags, LoRaMacEventInfo_t *info )
{
    switch( info->RxPort ) // Check Rx port number
    {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( info->RxBufferSize == 1 )
            {
                AppLedStateOn = info->RxBuffer[0] & 0x01;
                Led3StateChanged = true;
            }
            break;
        case 224:
            if( ComplianceTestOn == false )
            {
                // Check compliance test enable command (i)
                if( ( info->RxBufferSize == 4 ) && 
                    ( info->RxBuffer[0] == 0x01 ) &&
                    ( info->RxBuffer[1] == 0x01 ) &&
                    ( info->RxBuffer[2] == 0x01 ) &&
                    ( info->RxBuffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTestDownLinkCounter = 0;
                    ComplianceTestLinkCheck = false;
                    ComplianceTestDemodMargin = 0;
                    ComplianceTestNbGateways = 0;
                    ComplianceTestOn = true;
                    ComplianceTestState = 1;
                    
                    LoRaMacSetAdrOn( true );
#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                }
            }
            else
            {
                ComplianceTestState = info->RxBuffer[0];
                switch( ComplianceTestState )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    if( IsTxConfirmed == true )
                    {
                        AppDataSize = 6;
                    }
                    else
                    {
                        AppDataSize = 1;
                    }
                    ComplianceTestDownLinkCounter = 0;
                    ComplianceTestOn = false;
                    
                    LoRaMacSetAdrOn( LORAWAN_ADR_ON );
#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTestState = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTestState = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = info->RxBufferSize;

                    AppData[0] = 4;
                    for( uint8_t i = 1; i < AppDataSize; i++ )
                    {
                        AppData[i] = info->RxBuffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    LoRaMacLinkCheckReq( );
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
    }
}

static bool SendFrame( void )
{
    uint8_t sendFrameStatus = 0;

    LoRaMacUplinkStatus.Acked = false;
    LoRaMacUplinkStatus.Port = AppPort;
    LoRaMacUplinkStatus.Buffer = AppData;
    LoRaMacUplinkStatus.BufferSize = AppDataSize;
    
    SerialDisplayUpdateFrameType( IsTxConfirmed );

    if( IsTxConfirmed == false )
    {
        sendFrameStatus = LoRaMacSendFrame( AppPort, AppData, AppDataSize );
    }
    else
    {
        sendFrameStatus = LoRaMacSendConfirmedFrame( AppPort, AppData, AppDataSize, 8 );
    }

    switch( sendFrameStatus )
    {
    case 5: // NO_FREE_CHANNEL
        // Try again later
        return true;
    default:
        return false;
    }
}

#if( OVER_THE_AIR_ACTIVATION != 0 )

/*!
 * \brief Function executed on JoinReq Timeout event
 */
static void OnJoinReqTimerEvent( void )
{
    TimerStop( &JoinReqTimer );
    TxNextPacket = true;
}

#endif

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    TimerStop( &TxNextPacketTimer );
    TxNextPacket = true;
}

/*!
 * \brief Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void )
{
    TimerStop( &Led1Timer );
    Led1State = false;
    Led1StateChanged = true;
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void )
{
    TimerStop( &Led2Timer );
    Led2State = false;
    Led2StateChanged = true;
}

/*!
 * \brief Function to be executed on MAC layer event
 */
static void OnMacEvent( LoRaMacEventFlags_t *flags, LoRaMacEventInfo_t *info )
{
    if( flags->Bits.JoinAccept == 1 )
    {
#if( OVER_THE_AIR_ACTIVATION != 0 )
        TimerStop( &JoinReqTimer );
#endif
        IsNetworkJoined = true;
        IsNetworkJoinedStatusUpdate = true;
    }
    else
    {
        if( flags->Bits.Tx == 1 )
        {
        }

        if( flags->Bits.Rx == 1 )
        {
            if( ComplianceTestOn == true )
            {
                ComplianceTestDownLinkCounter++;
                if( flags->Bits.LinkCheck == 1 )
                {
                    ComplianceTestLinkCheck = true;
                    ComplianceTestDemodMargin = info->DemodMargin;
                    ComplianceTestNbGateways = info->NbGateways;
                }
            }
            if( flags->Bits.RxData == true )
            {
                ProcessRxFrame( flags, info );
            }
            
            LoRaMacDownlinkStatus.Rssi = info->RxRssi;
            if( info->RxSnr & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                LoRaMacDownlinkStatus.Snr = ( ( ~info->RxSnr + 1 ) & 0xFF ) >> 2;
                LoRaMacDownlinkStatus.Snr = -LoRaMacDownlinkStatus.Snr;
            }
            else
            {
                // Divide by 4
                LoRaMacDownlinkStatus.Snr = ( info->RxSnr & 0xFF ) >> 2;
            }
            LoRaMacDownlinkStatus.DownlinkCounter++;
            LoRaMacDownlinkStatus.RxData = flags->Bits.RxData;
            LoRaMacDownlinkStatus.Port = info->RxPort;
            LoRaMacDownlinkStatus.Buffer = info->RxBuffer;
            LoRaMacDownlinkStatus.BufferSize = info->RxBufferSize;
            
            Led2State = true;
            Led2StateChanged = true;
            TimerStart( &Led2Timer );
        }
        
        LoRaMacUplinkStatus.Acked = info->TxAckReceived;
        LoRaMacUplinkStatus.Datarate = info->TxDatarate;
        LoRaMacUplinkStatus.UplinkCounter = LoRaMacGetUpLinkCounter( ) - 1;
    }

    LinkStatusUpdated = true;
    // Schedule a new transmission
    ScheduleNextTx = true;
}

/**
 * Main application entry point.
 */
int main( void )
{
    SerialDisplayInit( );
    
#if( OVER_THE_AIR_ACTIVATION != 0 )
    uint8_t sendFrameStatus = 0;
#endif
    bool trySendingFrameAgain = false;

    BoardInit( );

    LoRaMacCallbacks.MacEvent = OnMacEvent;
    LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
    LoRaMacInit( &LoRaMacCallbacks );

    IsNetworkJoined = false;

    SerialDisplayUpdateActivationMode( OVER_THE_AIR_ACTIVATION );
    
#if( OVER_THE_AIR_ACTIVATION == 0 )
    LoRaMacInitNwkIds( LORAWAN_NETWORK_ID, LORAWAN_DEVICE_ADDRESS, NwkSKey, AppSKey );
    IsNetworkJoined = true;
    
    SerialDisplayUpdateNwkId( LORAWAN_NETWORK_ID );
    SerialDisplayUpdateDevAddr( LORAWAN_DEVICE_ADDRESS );
    SerialDisplayUpdateKey( 12, NwkSKey );
    SerialDisplayUpdateKey( 13, AppSKey );
#else
    // Sends a JoinReq Command every 5 seconds until the network is joined
    TimerInit( &JoinReqTimer, OnJoinReqTimerEvent ); 
    TimerSetValue( &JoinReqTimer, OVER_THE_AIR_ACTIVATION_DUTYCYCLE );
    
    SerialDisplayUpdateEui( 5, DevEui );
    SerialDisplayUpdateEui( 6, AppEui );
    SerialDisplayUpdateKey( 7, AppKey );

#endif

    SerialDisplayUpdateNetworkIsJoined( IsNetworkJoined );

    TxNextPacket = true;
    TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );
    
    TimerInit( &Led1Timer, OnLed1TimerEvent );
    TimerSetValue( &Led1Timer, 500000 );

    TimerInit( &Led2Timer, OnLed2TimerEvent );
    TimerSetValue( &Led2Timer, 500000 );

    LoRaMacSetAdrOn( LORAWAN_ADR_ON );
#if defined( USE_BAND_868 )
    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
    LoRaMacSetPublicNetwork( LORAWAN_PUBLIC_NETWORK );

    SerialDisplayUpdateAdr( LORAWAN_ADR_ON );
#if defined( USE_BAND_868 )
    SerialDisplayUpdateDutyCycle( LORAWAN_DUTYCYCLE_ON );
#else
    SerialDisplayUpdateDutyCycle( false );
#endif
    SerialDisplayUpdatePublicNetwork( LORAWAN_PUBLIC_NETWORK );
    
    LoRaMacDownlinkStatus.DownlinkCounter = 0;

    while( 1 )
    {
        while( IsNetworkJoined == false )
        {
#if( OVER_THE_AIR_ACTIVATION != 0 )
            if( TxNextPacket == true )
            {
                TxNextPacket = false;
                
                sendFrameStatus = LoRaMacJoinReq( DevEui, AppEui, AppKey );
                switch( sendFrameStatus )
                {
                case 1: // BUSY
                    break;
                case 0: // OK
                case 2: // NO_NETWORK_JOINED
                case 3: // LENGTH_PORT_ERROR
                case 4: // MAC_CMD_ERROR
                case 6: // DEVICE_OFF
                default:
                    // Relaunch timer for next trial
                    TimerStart( &JoinReqTimer );
                    break;
                }
            }
            SerialRxProcess( );
#endif
        }

        SerialRxProcess( );
        
        if( IsNetworkJoinedStatusUpdate == true )
        {
            IsNetworkJoinedStatusUpdate = false;
            SerialDisplayUpdateNetworkIsJoined( IsNetworkJoined );
        }
        if( Led1StateChanged == true )
        {
            Led1StateChanged = false;
            SerialDisplayUpdateLedState( 1, Led1State );
        }
        if( Led2StateChanged == true )
        {
            Led2StateChanged = false;
            SerialDisplayUpdateLedState( 2, Led2State );
        }
        if( Led3StateChanged == true )
        {
            Led3StateChanged = false;
            SerialDisplayUpdateLedState( 3, AppLedStateOn );
        }
        if( LinkStatusUpdated == true )
        {
            LinkStatusUpdated = false;
            SerialDisplayUpdateLedState( 2, Led2State );
            SerialDisplayUpdateUplink( LoRaMacUplinkStatus.Acked, LoRaMacUplinkStatus.Datarate, LoRaMacUplinkStatus.UplinkCounter, LoRaMacUplinkStatus.Port, LoRaMacUplinkStatus.Buffer, LoRaMacUplinkStatus.BufferSize );
            SerialDisplayUpdateDownlink( LoRaMacDownlinkStatus.RxData, LoRaMacDownlinkStatus.Rssi, LoRaMacDownlinkStatus.Snr, LoRaMacDownlinkStatus.DownlinkCounter, LoRaMacDownlinkStatus.Port, LoRaMacDownlinkStatus.Buffer, LoRaMacDownlinkStatus.BufferSize );
        }

        if( ScheduleNextTx == true )
        {
            ScheduleNextTx = false;
 
//            if( ComplianceTestOn == true )
//            {
//                TxNextPacket = true;
//            }
//            else
//            {
                // Schedule next packet transmission
                TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );
//            }
        }

        if( trySendingFrameAgain == true )
        {
            trySendingFrameAgain = SendFrame( );
        }
        if( TxNextPacket == true )
        {
            TxNextPacket = false;
        
            SerialDisplayUpdateDonwlinkRxData( false );
            
            PrepareTxFrame( AppPort );
            
            Led1State = true;
            SerialDisplayUpdateLedState( 1, Led1State );
            TimerStart( &Led1Timer );

            trySendingFrameAgain = SendFrame( );
            
            SerialDisplayUpdateUplink( LoRaMacUplinkStatus.Acked, LoRaMacUplinkStatus.Datarate, LoRaMacUplinkStatus.UplinkCounter, LoRaMacUplinkStatus.Port, LoRaMacUplinkStatus.Buffer, LoRaMacUplinkStatus.BufferSize );
        }
    }
}
