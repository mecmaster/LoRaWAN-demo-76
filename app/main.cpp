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
#define LORAWAN_DEVICE_EUI                          { 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11 }

/*!
 * Application IEEE EUI
 *
 * \remark must be written as a little endian value (reverse order of normal reading)
 */
#define LORAWAN_APPLICATION_EUI                     { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }

/*!
 * AES encryption/decryption cipher application key
 */
#define LORAWAN_APPLICATION_KEY                     { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00 }

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
#define LORAWAN_DEVICE_ADDRESS                      ( uint32_t )0x12345678

/*!
 * AES encryption/decryption cipher network session key
 */
#define LORAWAN_NWKSKEY                             { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }

/*!
 * AES encryption/decryption cipher application session key
 */
#define LORAWAN_APPSKEY                             { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }

#endif

/*!
 * Defines the application data transmission duty cycle
 */
#define APP_TX_DUTYCYCLE                            5000000  // 5 [s] value in us
#define APP_TX_DUTYCYCLE_RND                        1000000  // 1 [s] value in us

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

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        false

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
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_SIZE];

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

TimerEvent_t TxNextPacketTimer;

#if( OVER_THE_AIR_ACTIVATION != 0 )

/*!
 * Defines the join request timer
 */
TimerEvent_t JoinReqTimer;

#endif

/*!
 * Indicates if a new packet can be sent
 */
static bool TxNextPacket = true;
static bool ScheduleNextTx = false;

static bool AppLedStateOn = false;

static LoRaMacEvent_t LoRaMacEvents;

TimerEvent_t Led1Timer;
volatile bool Led1StateChanged = false;

TimerEvent_t Led2Timer;
volatile bool Led2StateChanged = false;

volatile bool Led3StateChanged = false;

volatile bool LinkStatusUpdated = false;

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
    SerialDisplayUpdateDutyCycle( LORAWAN_DUTYCYCLE_ON );
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
    AppData[0] = AppLedStateOn;
#if ( LORAWAN_CONFIRMED_MSG_ON == true )
    AppData[1] = LoRaMacDownlinkStatus.DownlinkCounter >> 8;
    AppData[2] = LoRaMacDownlinkStatus.DownlinkCounter;
    AppData[3] = LoRaMacDownlinkStatus.Rssi >> 8;
    AppData[4] = LoRaMacDownlinkStatus.Rssi;
    AppData[5] = LoRaMacDownlinkStatus.Snr;
#endif
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
        default:
            break;
    }
}

static bool SendFrame( void )
{
    uint8_t sendFrameStatus = 0;

    LoRaMacUplinkStatus.Acked = false;
    LoRaMacUplinkStatus.Port = LORAWAN_APP_PORT;
    LoRaMacUplinkStatus.Buffer = AppData;
    LoRaMacUplinkStatus.BufferSize = LORAWAN_APP_DATA_SIZE;
    
    SerialDisplayUpdateFrameType( LORAWAN_CONFIRMED_MSG_ON );
#if( LORAWAN_CONFIRMED_MSG_ON == false )
    sendFrameStatus = LoRaMacSendFrame( LORAWAN_APP_PORT, AppData, LORAWAN_APP_DATA_SIZE );
#else
    sendFrameStatus = LoRaMacSendConfirmedFrame( LORAWAN_APP_PORT, AppData, LORAWAN_APP_DATA_SIZE, 8 );
#endif
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
    Led1StateChanged = true;
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void )
{
    TimerStop( &Led2Timer );
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
            
            LoRaMacUplinkStatus.Acked = info->TxAckReceived;
            LoRaMacUplinkStatus.Datarate = info->TxDatarate;
            LoRaMacUplinkStatus.UplinkCounter = LoRaMacGetUpLinkCounter( ) - 1;
            
            LinkStatusUpdated = true;
            TimerStart( &Led2Timer );
        }
    }
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

    LoRaMacEvents.MacEvent = OnMacEvent;
    LoRaMacInit( &LoRaMacEvents, &BoardGetBatterieLevel );

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
    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
    LoRaMacSetPublicNetwork( LORAWAN_PUBLIC_NETWORK );

    SerialDisplayUpdateAdr( LORAWAN_ADR_ON );
    SerialDisplayUpdateDutyCycle( LORAWAN_DUTYCYCLE_ON );
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
            SerialDisplayUpdateLedState( 1, 0 );
        }
        if( Led2StateChanged == true )
        {
            Led2StateChanged = false;
            SerialDisplayUpdateLedState( 2, 0 );
        }
        if( Led3StateChanged == true )
        {
            Led3StateChanged = false;
            SerialDisplayUpdateLedState( 3, AppLedStateOn );
        }
        if( LinkStatusUpdated == true )
        {
            LinkStatusUpdated = false;
            SerialDisplayUpdateLedState( 2, 1 );
            SerialDisplayUpdateUplink( LoRaMacUplinkStatus.Acked, LoRaMacUplinkStatus.Datarate, LoRaMacUplinkStatus.UplinkCounter, LoRaMacUplinkStatus.Port, LoRaMacUplinkStatus.Buffer, LoRaMacUplinkStatus.BufferSize );
            SerialDisplayUpdateDownlink( LoRaMacDownlinkStatus.RxData, LoRaMacDownlinkStatus.Rssi, LoRaMacDownlinkStatus.Snr, LoRaMacDownlinkStatus.DownlinkCounter, LoRaMacDownlinkStatus.Port, LoRaMacDownlinkStatus.Buffer, LoRaMacDownlinkStatus.BufferSize );
        }

        if( ScheduleNextTx == true )
        {
            ScheduleNextTx = false;
            // Schedule next packet transmission
            TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
            TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
            TimerStart( &TxNextPacketTimer );
        }

        if( trySendingFrameAgain == true )
        {
            trySendingFrameAgain = SendFrame( );
        }
        if( TxNextPacket == true )
        {
            TxNextPacket = false;
        
            SerialDisplayUpdateDonwlinkRxData( false );
            
            PrepareTxFrame( LORAWAN_APP_PORT );
            
            SerialDisplayUpdateLedState( 1, 1 );
            TimerStart( &Led1Timer );

            trySendingFrameAgain = SendFrame( );
            
            SerialDisplayUpdateUplink( LoRaMacUplinkStatus.Acked, LoRaMacUplinkStatus.Datarate, LoRaMacUplinkStatus.UplinkCounter, LoRaMacUplinkStatus.Port, LoRaMacUplinkStatus.Buffer, LoRaMacUplinkStatus.BufferSize );
        }
    }
}
