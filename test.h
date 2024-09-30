#include <stdlib.h>
#include "pico/stdlib.h"

enum bit_order{
    LSBFIRST,
    MSBFIRST
};
typedef enum bit_order bit_order;

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t val){
    //write the bits to the data pin
    for(int i = 0; i < 8; i++) {
        gpio_put(clockPin, 0);

        gpio_put(dataPin, (val & 0x80) ? 1 : 0 );
        val <<= 1;

        gpio_put(clockPin, 1);
    }
}