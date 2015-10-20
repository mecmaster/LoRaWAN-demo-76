/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: VT100 terminal support class

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __VT100_H__
#define __VT100_H__

class VT100 : public Serial
{
public:
    enum TextAttributes
    {
        ATTR_OFF      = 0,
        BOLD          = 1,
        USCORE        = 4,
        BLINK         = 5,
        REVERSE       = 7,
        BOLD_OFF      = 21,
        USCORE_OFF    = 24,
        BLINK_OFF     = 25,
        REVERSE_OFF   = 27,
    };

    enum Colors
    {
        BLACK   = 0,
        RED     = 1,
        GREEN   = 2,
        BROWN   = 3,
        BLUE    = 4,
        MAGENTA = 5,
        CYAN    = 6,
        WHITE   = 7,
    };
    /*!
     *
     */
    VT100( PinName tx, PinName rx ): Serial( tx, rx )
    {
        this->baud( 115200 );
        // initializes terminal to "power-on" settings
        // ESC c
        this->printf( "\x1B\x63" );
    }
    
    void ClearScreen( uint8_t param )
    {
        // ESC [ Ps J
        // 0    Clear screen from cursor down
        // 1    Clear screen from cursor up
        // 2    Clear entire screen 

        this->printf( "\x1B[%dJ", param );
    }

    void ClearLine( uint8_t param )
    {
        // ESC [ Ps K
        // 0    Erase from the active position to the end of the line, inclusive (default)
        // 1    Erase from the start of the screen to the active position, inclusive
        // 2    Erase all of the line, inclusive

        this->printf( "\x1B[%dK", param );
    }

    void SetAttribute( uint8_t attr )
    {
        // ESC [ Ps;...;Ps m
        this->printf( "\x1B[%dm", attr );
    }

    void SetAttribute( uint8_t attr, uint8_t fgcolor, uint8_t bgcolor )
    {
        // ESC [ Ps;...;Ps m
        this->printf( "\x1B[%d;%d;%dm", attr, fgcolor + 30, bgcolor + 40 );
    }

    void SetCursorMode( uint8_t visible )
    {
        if( visible == true )
        {
            // ESC [ ? 25 h
            this->printf( "\x1B[?25h" );
        }
        else
        {
            // ESC [ ? 25 l
            this->printf( "\x1B[?25l" );
        }
    }

    void SetCursorPos( uint8_t line, uint8_t col )
    {
        // ESC [ Pl ; Pc H
        this->printf( "\x1B[%d;%dH", line, col );
    }

    void PutStringAt( uint8_t line, uint8_t col, const char *s )
    {
        this->SetCursorPos( line, col );
        this->printf( "%s", s );
    }

    void PutCharAt( uint8_t line, uint8_t col, uint8_t c )
    {
        this->SetCursorPos( line, col );
        this->printf( "%c", c );
    }

    void PutHexAt( uint8_t line, uint8_t col, uint16_t n )
    {
        this->SetCursorPos( line, col );
        this->printf( "%X", n );
    }

    void PutBoxDrawingChar( uint8_t c )
    {
        this->printf( "\x1B(0%c\x1b(B", c );
    }
    
    bool Readable( void )
    {
        return this->readable( );
    }
    
    uint8_t GetChar( void )
    {
        return this->getc( );
    }
private:
    
};

#endif // __VT100_H__
