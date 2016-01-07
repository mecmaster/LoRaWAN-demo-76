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
#include "Comissioning.h"
#include "SerialDisplay.h"

/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           10000000  // 10 [s] value in us

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
 * Default mote datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    true

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

#if defined( USE_BAND_868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

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

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

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

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;
volatile bool Led3StateChanged = false;
/*!
 * Timer to handle the state of LED1
 */
static TimerEvent_t Led1Timer;
volatile bool Led1State = false;
volatile bool Led1StateChanged = false;
/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;
volatile bool Led2State = false;
volatile bool Led2StateChanged = false;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Device states
 */
static enum eDevicState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

/*
 * SerialDisplay managment variables
 */

/*!
 * Indicates if the MAC layer network join status has changed.
 */
static bool IsNetworkJoinedStatusUpdate = false;

/*!
 * Strucure containing the Uplink status
 */
struct sLoRaMacUplinkStatus
{
    uint8_t Acked;
    int8_t Datarate;
    uint16_t UplinkCounter;
    uint8_t Port;
    uint8_t *Buffer;
    uint8_t BufferSize;
}LoRaMacUplinkStatus;
volatile bool UplinkStatusUpdated = false;

/*!
 * Strucure containing the Downlink status
 */
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
volatile bool DownlinkStatusUpdated = false;

void SerialDisplayRefresh( void )
{
    MibRequestConfirm_t mibReq;

    SerialDisplayInit( );
    SerialDisplayUpdateActivationMode( OVER_THE_AIR_ACTIVATION );

#if( OVER_THE_AIR_ACTIVATION == 0 )
    SerialDisplayUpdateNwkId( LORAWAN_NETWORK_ID );
    SerialDisplayUpdateDevAddr( DevAddr );
    SerialDisplayUpdateKey( 12, NwkSKey );
    SerialDisplayUpdateKey( 13, AppSKey );
#else
    SerialDisplayUpdateEui( 5, DevEui );
    SerialDisplayUpdateEui( 6, AppEui );
    SerialDisplayUpdateKey( 7, AppKey );
#endif

    mibReq.Type = MIB_NETWORK_JOINED;
    LoRaMacMibGetRequestConfirm( &mibReq );
    SerialDisplayUpdateNetworkIsJoined( mibReq.Param.IsNetworkJoined );

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
 * \brief   Prepares the payload of the frame
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
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    
    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        
        LoRaMacUplinkStatus.Acked = false;
        LoRaMacUplinkStatus.Port = 0;
        LoRaMacUplinkStatus.Buffer = NULL;
        LoRaMacUplinkStatus.BufferSize = 0;
        SerialDisplayUpdateFrameType( false );
    }
    else
    {
        LoRaMacUplinkStatus.Acked = false;
        LoRaMacUplinkStatus.Port = AppPort;
        LoRaMacUplinkStatus.Buffer = AppData;
        LoRaMacUplinkStatus.BufferSize = AppDataSize;
        SerialDisplayUpdateFrameType( IsTxConfirmed );

        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.nbRetries = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
}

/*!
 * \brief Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void )
{
    TimerStop( &Led1Timer );
    // Switch LED 1 OFF
    Led1State = false;
    Led1StateChanged = true;
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void )
{
    TimerStop( &Led2Timer );
    // Switch LED 2 OFF
    Led2State = false;
    Led2StateChanged = true;
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] McpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *McpsConfirm )
{
    if( McpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( McpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbRetries
                LoRaMacUplinkStatus.Acked = McpsConfirm->AckReceived;
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
        LoRaMacUplinkStatus.Datarate = McpsConfirm->Datarate;
        LoRaMacUplinkStatus.UplinkCounter = McpsConfirm->UpLinkCounter;
    
        UplinkStatusUpdated = true;
    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] McpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *McpsIndication )
{
    if( McpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( McpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    LoRaMacDownlinkStatus.Rssi = McpsIndication->Rssi;
    if( McpsIndication->Snr & 0x80 ) // The SNR sign bit is 1
    {
        // Invert and divide by 4
        LoRaMacDownlinkStatus.Snr = ( ( ~McpsIndication->Snr + 1 ) & 0xFF ) >> 2;
        LoRaMacDownlinkStatus.Snr = -LoRaMacDownlinkStatus.Snr;
    }
    else
    {
        // Divide by 4
        LoRaMacDownlinkStatus.Snr = ( McpsIndication->Snr & 0xFF ) >> 2;
    }
    LoRaMacDownlinkStatus.DownlinkCounter++;
    LoRaMacDownlinkStatus.RxData = McpsIndication->RxData;
    LoRaMacDownlinkStatus.Port = McpsIndication->Port;
    LoRaMacDownlinkStatus.Buffer = McpsIndication->Buffer;
    LoRaMacDownlinkStatus.BufferSize = McpsIndication->BufferSize;

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( McpsIndication->RxData == true )
    {
        switch( McpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( McpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = McpsIndication->Buffer[0] & 0x01;
                Led3StateChanged = true;
            }
            break;
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( McpsIndication->BufferSize == 4 ) && 
                    ( McpsIndication->Buffer[0] == 0x01 ) &&
                    ( McpsIndication->Buffer[1] == 0x01 ) &&
                    ( McpsIndication->Buffer[2] == 0x01 ) &&
                    ( McpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;
                    
                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                }
            }
            else
            {
                ComplianceTest.State = McpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;
                    
                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = McpsIndication->BufferSize;

                    AppData[0] = 4;
                    for( uint8_t i = 1; i < AppDataSize; i++ )
                    {
                        AppData[i] = McpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
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

    // Switch LED 2 ON for each received downlink
    Led2State = true;
    Led2StateChanged = true;
    TimerStart( &Led2Timer );
    DownlinkStatusUpdated = true;
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] MlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *MlmeConfirm )
{
    if( MlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( MlmeConfirm->MlmeRequest )
        {
            case MLME_JOIN:
            {
                // Status is OK, node has joined the network
                IsNetworkJoinedStatusUpdate = true;
                break;
            }
            case MLME_LINK_CHECK:
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = MlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = MlmeConfirm->NbGateways;
                }
                break;
            }
            default:
                break;
        }
    }
    NextTx = true;
    UplinkStatusUpdated = true;
}

/**
 * Main application entry point.
 */
int main( void )
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    BoardInit( );
    SerialDisplayInit( );

    DeviceState = DEVICE_STATE_INIT;

    while( 1 )
    {
        SerialRxProcess( );
        if( IsNetworkJoinedStatusUpdate == true )
        {
            IsNetworkJoinedStatusUpdate = false;
            mibReq.Type = MIB_NETWORK_JOINED;
            LoRaMacMibGetRequestConfirm( &mibReq );
            SerialDisplayUpdateNetworkIsJoined( mibReq.Param.IsNetworkJoined );
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
        if( UplinkStatusUpdated == true )
        {
            UplinkStatusUpdated = false;
            SerialDisplayUpdateUplink( LoRaMacUplinkStatus.Acked, LoRaMacUplinkStatus.Datarate, LoRaMacUplinkStatus.UplinkCounter, LoRaMacUplinkStatus.Port, LoRaMacUplinkStatus.Buffer, LoRaMacUplinkStatus.BufferSize );
        }
        if( DownlinkStatusUpdated == true )
        {
            DownlinkStatusUpdated = false;
            SerialDisplayUpdateLedState( 2, Led2State );
            SerialDisplayUpdateDownlink( LoRaMacDownlinkStatus.RxData, LoRaMacDownlinkStatus.Rssi, LoRaMacDownlinkStatus.Snr, LoRaMacDownlinkStatus.DownlinkCounter, LoRaMacDownlinkStatus.Port, LoRaMacDownlinkStatus.Buffer, LoRaMacDownlinkStatus.BufferSize );
        }
        
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );

                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                TimerInit( &Led1Timer, OnLed1TimerEvent );
                TimerSetValue( &Led1Timer, 25000 );

                TimerInit( &Led2Timer, OnLed2TimerEvent );
                TimerSetValue( &Led2Timer, 25000 );

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
                LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
                SerialDisplayUpdateDutyCycle( LORAWAN_DUTYCYCLE_ON );
#endif
                SerialDisplayUpdateActivationMode( OVER_THE_AIR_ACTIVATION );
                SerialDisplayUpdateAdr( LORAWAN_ADR_ON );
                SerialDisplayUpdatePublicNetwork( LORAWAN_PUBLIC_NETWORK );

                LoRaMacDownlinkStatus.DownlinkCounter = 0;

                DeviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
#if( OVER_THE_AIR_ACTIVATION != 0 )
                MlmeReq_t mlmeReq;

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;

                if( NextTx == true )
                {
                    LoRaMacMlmeRequest( &mlmeReq );
                }

                SerialDisplayUpdateEui( 5, DevEui );
                SerialDisplayUpdateEui( 6, AppEui );
                SerialDisplayUpdateKey( 7, AppKey );

                // Schedule next packet transmission
                TxDutyCycleTime = OVER_THE_AIR_ACTIVATION_DUTYCYCLE;
                DeviceState = DEVICE_STATE_CYCLE;

#else
                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NWK_SKEY;
                mibReq.Param.NwkSKey = NwkSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_APP_SKEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NETWORK_JOINED;
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm( &mibReq );

                SerialDisplayUpdateNwkId( LORAWAN_NETWORK_ID );
                SerialDisplayUpdateDevAddr( DevAddr );
                SerialDisplayUpdateKey( 12, NwkSKey );
                SerialDisplayUpdateKey( 13, AppSKey );

                DeviceState = DEVICE_STATE_SEND;
#endif
                IsNetworkJoinedStatusUpdate = true;
                break;
            }
            case DEVICE_STATE_SEND:
            {
                if( NextTx == true )
                {
                    SerialDisplayUpdateUplinkAcked( false );
                    SerialDisplayUpdateDonwlinkRxData( false );
                    PrepareTxFrame( AppPort );

                    NextTx = SendFrame( );

                    // Switch LED 1 ON
                    Led1State = true;
                    Led1StateChanged = true;
                    TimerStart( &Led1Timer );
                }
                if( ComplianceTest.Running == true )
                {
                    // Schedule next packet transmission as soon as possible
                    TxDutyCycleTime = 1000; // 1 ms
                }
                else
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                }
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                // Schedule next packet transmission
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );

                DeviceState = DEVICE_STATE_SLEEP;
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                // Wake up through events
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }
}
