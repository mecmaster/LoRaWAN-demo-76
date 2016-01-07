/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: VT100 serial display management

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "vt100.h"
#include "SerialDisplay.h"

VT100 vt( USBTX, USBRX );

void SerialPrintCheckBox( bool activated, uint8_t color )
{
    if( activated == true )
    {
        vt.SetAttribute( VT100::ATTR_OFF, color, color );
    }
    else
    {
        vt.SetAttribute( VT100::ATTR_OFF );
    }
    vt.printf( " " );
    vt.SetAttribute( VT100::ATTR_OFF );
}

void SerialDisplayUpdateActivationMode( bool otaa )
{
    vt.SetCursorPos( 4, 17 );
    SerialPrintCheckBox( otaa, VT100::WHITE );
    vt.SetCursorPos( 9, 17 );
    SerialPrintCheckBox( !otaa, VT100::WHITE );
}

void SerialDisplayUpdateEui( uint8_t line, uint8_t *eui )
{
    vt.SetCursorPos( line, 27 );
    for( uint8_t i = 0; i < 8; i++ )
    {
        vt.printf( "%02X ", eui[i] );
    }
    vt.SetCursorPos( line, 50 );
    vt.printf( "]" );
}

void SerialDisplayUpdateKey( uint8_t line, uint8_t *key )
{
    vt.SetCursorPos( line, 27 );
    for( uint8_t i = 0; i < 16; i++ )
    {
        vt.printf( "%02X ", key[i] );
    }
    vt.SetCursorPos( line, 74 );
    vt.printf( "]" );
}

void SerialDisplayUpdateNwkId( uint8_t id )
{
    vt.SetCursorPos( 10, 27 );
    vt.printf( "%03d", id );
}

void SerialDisplayUpdateDevAddr( uint32_t addr )
{
    vt.SetCursorPos( 11, 27 );
    vt.printf( "%02X %02X %02X %02X", ( addr >> 24 ) & 0xFF, ( addr >> 16 ) & 0xFF, ( addr >> 8 ) & 0xFF, addr & 0xFF );
}

void SerialDisplayUpdateFrameType( bool confirmed )
{
    vt.SetCursorPos( 15, 17 );
    SerialPrintCheckBox( confirmed, VT100::WHITE );
    vt.SetCursorPos( 15, 32 );
    SerialPrintCheckBox( !confirmed, VT100::WHITE );
}

void SerialDisplayUpdateAdr( bool adr )
{
    vt.SetCursorPos( 16, 27 );
    if( adr == true )
    {
        vt.printf( " ON" );
    }
    else
    {
        vt.printf( "OFF" );
    }
}

void SerialDisplayUpdateDutyCycle( bool dutyCycle )
{
    vt.SetCursorPos( 17, 27 );
    if( dutyCycle == true )
    {
        vt.printf( " ON" );
    }
    else
    {
        vt.printf( "OFF" );
    }
}

void SerialDisplayUpdatePublicNetwork( bool network )
{
    vt.SetCursorPos( 19, 17 );
    SerialPrintCheckBox( network, VT100::WHITE );
    vt.SetCursorPos( 19, 30 );
    SerialPrintCheckBox( !network, VT100::WHITE );
}

void SerialDisplayUpdateNetworkIsJoined( bool state )
{
    vt.SetCursorPos( 20, 17 );
    SerialPrintCheckBox( !state, VT100::RED );
    vt.SetCursorPos( 20, 30 );
    SerialPrintCheckBox( state, VT100::GREEN );
}

void SerialDisplayUpdateLedState( uint8_t id, uint8_t state )
{
    switch( id )
    {
        case 1:
            vt.SetCursorPos( 22, 17 );
            SerialPrintCheckBox( state, VT100::RED );
            break;
        case 2:
            vt.SetCursorPos( 22, 31 );
            SerialPrintCheckBox( state, VT100::GREEN );
            break;
        case 3:
            vt.SetCursorPos( 22, 45 );
            SerialPrintCheckBox( state, VT100::BLUE );
            break;
    }
}

void SerialDisplayUpdateData( uint8_t line, uint8_t *buffer, uint8_t size )
{
    if( size != 0 )
    {
        vt.SetCursorPos( line, 27 );
        for( uint8_t i = 0; i < size; i++ )
        {
            vt.printf( "%02X ", buffer[i] );
            if( ( ( i + 1 ) % 16 ) == 0 )
            {
                line++;
                vt.SetCursorPos( line, 27 );
            }
        }
        for( uint8_t i = size; i < 64; i++ )
        {
            vt.printf( "__ " );
            if( ( ( i + 1 ) % 16 ) == 0 )
            {
                line++;
                vt.SetCursorPos( line, 27 );
            }
        }
        vt.SetCursorPos( line - 1, 74 );
        vt.printf( "]" );
    }
    else
    {
        vt.SetCursorPos( line, 27 );
        for( uint8_t i = 0; i < 64; i++ )
        {
            vt.printf( "__ " );
            if( ( ( i + 1 ) % 16 ) == 0 )
            {
                line++;
                vt.SetCursorPos( line, 27 );
            }
        }
        vt.SetCursorPos( line - 1, 74 );
        vt.printf( "]" );
    }
}

void SerialDisplayUpdateUplinkAcked( bool state )
{
    vt.SetCursorPos( 24, 36 );
    SerialPrintCheckBox( state, VT100::GREEN );
}

void SerialDisplayUpdateUplink( bool acked, uint8_t datarate, uint16_t counter, uint8_t port, uint8_t *buffer, uint8_t bufferSize )
{
    // Acked
    SerialDisplayUpdateUplinkAcked( acked );
    // Datarate
    vt.SetCursorPos( 25, 33 );
    vt.printf( "DR%d", datarate );
    // Counter
    vt.SetCursorPos( 26, 27 );
    vt.printf( "%10d", counter );
    // Port
    vt.SetCursorPos( 27, 34 );
    vt.printf( "%3d", port );
    // Data
    SerialDisplayUpdateData( 28, buffer, bufferSize );
    // Help message
    vt.SetCursorPos( 42, 1 );
    vt.printf( "To refresh screen please hit 'r' key." );
}

void SerialDisplayUpdateDonwlinkRxData( bool state )
{
    vt.SetCursorPos( 34, 4 );
    SerialPrintCheckBox( state, VT100::GREEN );
}

void SerialDisplayUpdateDownlink( bool rxData, int16_t rssi, int8_t snr, uint16_t counter, uint8_t port, uint8_t *buffer, uint8_t bufferSize )
{
    // Rx data
    SerialDisplayUpdateDonwlinkRxData( rxData );
    // RSSI
    vt.SetCursorPos( 33, 32 );
    vt.printf( "%5d", rssi );
    // SNR
    vt.SetCursorPos( 34, 32 );
    vt.printf( "%5d", snr );
    // Counter
    vt.SetCursorPos( 35, 27 );
    vt.printf( "%10d", counter );
    if( rxData == true )
    {
        // Port
        vt.SetCursorPos( 36, 34 );
        vt.printf( "%3d", port );
        // Data
        SerialDisplayUpdateData( 37, buffer, bufferSize );
    }
    else
    {
        // Port
        vt.SetCursorPos( 36, 34 );
        vt.printf( "   " );
        // Data
        SerialDisplayUpdateData( 37, NULL, 0 );
    }
}

void SerialDisplayDrawFirstLine( void )
{
    vt.PutBoxDrawingChar( 'l' );
    for( int8_t i = 0; i <= 77; i++ )
    {
        vt.PutBoxDrawingChar( 'q' );
    }
    vt.PutBoxDrawingChar( 'k' );
    vt.printf( "\r\n" );
}

void SerialDisplayDrawTitle( const char* title )
{
    vt.PutBoxDrawingChar( 'x' );
    vt.printf( "%s", title );
    vt.PutBoxDrawingChar( 'x' );
    vt.printf( "\r\n" );
}
void SerialDisplayDrawTopSeparator( void )
{
    vt.PutBoxDrawingChar( 't' );
    for( int8_t i = 0; i <= 11; i++ )
    {
        vt.PutBoxDrawingChar( 'q' );
    }
    vt.PutBoxDrawingChar( 'w' );
    for( int8_t i = 0; i <= 64; i++ )
    {
        vt.PutBoxDrawingChar( 'q' );
    }
    vt.PutBoxDrawingChar( 'u' );
    vt.printf( "\r\n" );
}

void SerialDisplayDrawColSeparator( void )
{
    vt.PutBoxDrawingChar( 'x' );
    for( int8_t i = 0; i <= 11; i++ )
    {
        vt.PutBoxDrawingChar( ' ' );
    }
    vt.PutBoxDrawingChar( 't' );
    for( int8_t i = 0; i <= 64; i++ )
    {
        vt.PutBoxDrawingChar( 'q' );
    }
    vt.PutBoxDrawingChar( 'u' );
    vt.printf( "\r\n" );
}

void SerialDisplayDrawSeparator( void )
{
    vt.PutBoxDrawingChar( 't' );
    for( int8_t i = 0; i <= 11; i++ )
    {
        vt.PutBoxDrawingChar( 'q' );
    }
    vt.PutBoxDrawingChar( 'n' );
    for( int8_t i = 0; i <= 64; i++ )
    {
        vt.PutBoxDrawingChar( 'q' );
    }
    vt.PutBoxDrawingChar( 'u' );
    vt.printf( "\r\n" );
}

void SerialDisplayDrawLine( const char* firstCol, const char* secondCol )
{
    vt.PutBoxDrawingChar( 'x' );
    vt.printf( "%s", firstCol );
    vt.PutBoxDrawingChar( 'x' );
    vt.printf( "%s", secondCol );
    vt.PutBoxDrawingChar( 'x' );
    vt.printf( "\r\n" );
}

void SerialDisplayDrawBottomLine( void )
{
    vt.PutBoxDrawingChar( 'm' );
    for( int8_t i = 0; i <= 11; i++ )
    {
        vt.PutBoxDrawingChar( 'q' );
    }
    vt.PutBoxDrawingChar( 'v' );
    for( int8_t i = 0; i <= 64; i++ )
    {
        vt.PutBoxDrawingChar( 'q' );
    }
    vt.PutBoxDrawingChar( 'j' );
    vt.printf( "\r\n" );
}

void SerialDisplayInit( void )
{
    vt.ClearScreen( 2 );
    vt.SetCursorMode( false );
    vt.SetCursorPos( 0, 0 );
    
    // "+-----------------------------------------------------------------------------+" );
    SerialDisplayDrawFirstLine( );
    // "¦                      LoRaWAN Demonstration Application                      ¦" );
    SerialDisplayDrawTitle( "                      LoRaWAN Demonstration Application                       " );
    // "+------------+----------------------------------------------------------------¦" );
    SerialDisplayDrawTopSeparator( );
    // "¦ Activation ¦ [ ]Over The Air                                                ¦" );
    SerialDisplayDrawLine( " Activation ", " [ ]Over The Air                                                 " );
    // "¦            ¦ DevEui    [__ __ __ __ __ __ __ __]                            ¦" );
    SerialDisplayDrawLine( "            ", " DevEui    [__ __ __ __ __ __ __ __]                             " );
    // "¦            ¦ AppEui    [__ __ __ __ __ __ __ __]                            ¦" );
    SerialDisplayDrawLine( "            ", " AppEui    [__ __ __ __ __ __ __ __]                             " );
    // "¦            ¦ AppKey  [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __]      ¦" );
    SerialDisplayDrawLine( "            ", " AppKey    [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __]     " );
    // "¦            +----------------------------------------------------------------¦" );
    SerialDisplayDrawColSeparator( );
    // "¦            ¦ [x]Personalisation                                             ¦" );
    SerialDisplayDrawLine( "            ", " [ ]Personalisation                                              " );
    // "¦            ¦ NwkId     [___]                                                ¦" );
    SerialDisplayDrawLine( "            ", " NwkId     [___]                                                 " );
    // "¦            ¦ DevAddr   [__ __ __ __]                                        ¦" );
    SerialDisplayDrawLine( "            ", " DevAddr   [__ __ __ __]                                         " );
    // "¦            ¦ NwkSKey   [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __]    ¦" );
    SerialDisplayDrawLine( "            ", " NwkSKey   [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __]     " );
    // "¦            ¦ AppSKey   [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __]    ¦" );
    SerialDisplayDrawLine( "            ", " AppSKey   [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __]     " );
    // "+------------+----------------------------------------------------------------¦" );
    SerialDisplayDrawSeparator( );
    // "¦ MAC params ¦ [ ]Confirmed / [ ]Unconfirmed                                  ¦" );
    SerialDisplayDrawLine( " MAC params ", " [ ]Confirmed / [ ]Unconfirmed                                   " );
    // "¦            ¦ ADR       [   ]                                                ¦" );
    SerialDisplayDrawLine( "            ", " ADR       [   ]                                                 " );
    // "¦            ¦ Duty cycle[   ]                                                ¦" );
    SerialDisplayDrawLine( "            ", " Duty cycle[   ]                                                 " );
    // "+------------+----------------------------------------------------------------¦" );
    SerialDisplayDrawSeparator( );
    // "¦ Network    ¦ [ ]Public  / [ ]Private                                        ¦" );
    SerialDisplayDrawLine( " Network    ", " [ ]Public  / [ ]Private                                         " );
    // "¦            ¦ [ ]Joining / [ ]Joined                                         ¦" );
    SerialDisplayDrawLine( "            ", " [ ]Joining / [ ]Joined                                          " );
    // "+------------+----------------------------------------------------------------¦" );
    SerialDisplayDrawSeparator( );
    // "¦ LED status ¦ [ ]LED1(Tx) / [ ]LED2(Rx) / [ ]LED3(App)                       ¦" );
    SerialDisplayDrawLine( " LED status ", " [ ]LED1(Tx) / [ ]LED2(Rx) / [ ]LED3(App)                        " );
    // "+------------+----------------------------------------------------------------¦" );
    SerialDisplayDrawSeparator( );
    // "¦ Uplink     ¦ Acked              [ ]                                         ¦" );
    SerialDisplayDrawLine( " Uplink     ", " Acked              [ ]                                          " );
    // "¦            ¦ Datarate        [    ]                                         ¦" );
    SerialDisplayDrawLine( "            ", " Datarate        [    ]                                          " );
    // "¦            ¦ Counter   [          ]                                         ¦" );
    SerialDisplayDrawLine( "            ", " Counter   [          ]                                          " );
    // "¦            ¦ Port             [   ]                                         ¦" );
    SerialDisplayDrawLine( "            ", " Port             [   ]                                          " );
    // "¦            ¦ Data      [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __     ¦" );
    SerialDisplayDrawLine( "            ", " Data      [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    // "¦            ¦            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __     ¦" );
    SerialDisplayDrawLine( "            ", "            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    // "¦            ¦            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __     ¦" );
    SerialDisplayDrawLine( "            ", "            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    // "¦            ¦            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __     ¦" );
    SerialDisplayDrawLine( "            ", "            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    // "+------------+----------------------------------------------------------------¦" );
    SerialDisplayDrawSeparator( );
    // "¦ Downlink   ¦ RSSI           [     ] dBm                                     ¦" );
    SerialDisplayDrawLine( " Downlink   ", " RSSI           [     ] dBm                                      " );
    // "¦ [ ]Data    ¦ SNR      [     ] dB                                            ¦" );
    SerialDisplayDrawLine( " [ ]Data    ", " SNR            [     ] dB                                       " );
    // "¦            ¦ Counter  [          ]                                          ¦" );
    // "¦            ¦ Counter   [          ]                                         ¦" );
    SerialDisplayDrawLine( "            ", " Counter   [          ]                                          " );
    // "¦            ¦ Port             [   ]                                         ¦" );
    SerialDisplayDrawLine( "            ", " Port             [   ]                                          " );
    // "¦            ¦ Data      [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __     ¦" );
    SerialDisplayDrawLine( "            ", " Data      [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    // "¦            ¦            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __     ¦" );
    SerialDisplayDrawLine( "            ", "            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    // "¦            ¦            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __     ¦" );
    SerialDisplayDrawLine( "            ", "            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    // "¦            ¦            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __     ¦" );
    SerialDisplayDrawLine( "            ", "            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    // "+------------+----------------------------------------------------------------+" );
    SerialDisplayDrawBottomLine( );
    vt.printf( "To refresh screen please hit 'r' key.\r\n" );
}

bool SerialDisplayReadable( void )
{
    return vt.Readable( );
}

uint8_t SerialDisplayGetChar( void )
{
    return vt.GetChar( );
}
