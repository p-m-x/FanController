#include <font.h>
#include <avr/pgmspace.h>
class SDA5708 {
    uint8_t pinLoad;
    uint8_t pinData;
    uint8_t pinClock;
    uint8_t pinReset;

public:
    SDA5708(uint8_t pinLoad, uint8_t pinData, uint8_t pinClock, uint8_t pinReset)
        : pinLoad(pinLoad), pinData(pinData), pinClock(pinClock), pinReset(pinReset) {
        pinMode(pinLoad, OUTPUT);
        pinMode(pinData, OUTPUT);
        pinMode(pinClock, OUTPUT);
        pinMode(pinReset, OUTPUT);
    }

    void begin(void)
    {
        digitalWrite(pinLoad, HIGH);
        digitalWrite(pinReset, LOW);
        digitalWrite(pinReset, HIGH);
    }

    void brightness(uint8_t val)
    {
        sendByte(0b11100000 | (val & 0b00000111));
    }

    void sendByte(uint8_t byte)
    {
        uint8_t x;	
	    // LOAD auf Low
        digitalWrite(pinLoad, LOW);
	    // jede Byte-Stelle ans Display senden
        for (x=0; x<=7;x++)	{
            // DATA entsprechend setzen
            if ((byte>>x)&1) {
                digitalWrite(pinData, HIGH);
                } else {
                digitalWrite(pinData, LOW);
                }
                // SDCLK toggeln
                digitalWrite(pinClock, HIGH);
                digitalWrite(pinClock, LOW);
        }
        // LOAD auf High
        digitalWrite(pinLoad, HIGH);
    }

    void digit(uint8_t sign, uint8_t digit)
    {
        uint8_t i;
        if ((sign < 0x20) || (sign > 0x7f)) sign = 0x20;
        if (digit > 7) digit = 0;
        setCyrsor(digit);
        for (i = 0; i < 7; i++) {
            sendByte(pgm_read_byte(&font[(sign - 0x20) * 7 + i]) / 8);
        }
    }

    void setCyrsor(uint8_t cursor)
    {
        if (cursor > 7) cursor = 0;
        sendByte(0b10100000 | cursor);
    }

    void print(char *text)
    {
        uint8_t cursor=0;
        char *p=text;
        while (*p) {
            digit(*p, cursor);
            cursor++;
            p++;
        }
    }

    void printAt(char *text, uint8_t cursor)
    {
        if (cursor > 7) cursor = 0;
        char *p=text;
        while (*p) {
            digit(*p, cursor);
            cursor++;
            p++;
        }
    }

    void clear()
    {
        begin();
    }

};
