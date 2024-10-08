#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <time.h>

void blink_led(int num_times){
    int blink = 0;
    while(blink != num_times){
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(300);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(300);
        blink++;
    }
}

#define CLK_PIN 14
#define DATA_PIN 13
#define LATCH_PIN 15
#define ONE      0b0000000000000001
#define TWO      0b0000000000000011
#define THREE    0b0000000000000111
#define FOUR     0b0000000000001111
#define FIVE     0b0000000000011111
#define SIX      0b0000000000111111
#define SEVEN    0b0000000001111111
#define EIGHT    0b0000000011111111
#define NINE     0b0000000111111111
#define TEN      0b0000001111111111
#define ELEVEN   0b0000011111111111
#define TWELVE   0b0000111111111111
#define THIRTEEN 0b0001111111111111
#define FOURTEEN 0b0011111111111111
#define FIFTEEN  0b0111111111111111
#define SIXTEEN  0b1111111111111111

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t val){
    //write the bits to the data pin
    for(int i = 0; i < 8; i++) {
        gpio_put(clockPin, 0);

        gpio_put(dataPin, (val & 0x80) ? 1 : 0 );
        val <<= 1;

        gpio_put(clockPin, 1);
    }
}

void writeTo595(uint16_t _data ) {
  // Output low level to latchPin
  gpio_put(LATCH_PIN, 0);

  //Send higher byte (second register first)
  shiftOut(DATA_PIN, CLK_PIN, _data >> 8);

  // Send serial data to 74HC595
  shiftOut(DATA_PIN, CLK_PIN, _data & 0xFF);
  
  // Output high level to latchPin, and 74HC595 will update the data to the parallel output port.
  gpio_put(LATCH_PIN, 1);

}

void main() {
    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return;
    }                                                                             
    srand(time(NULL));
    // Set clk pin as output
    gpio_init(CLK_PIN);
    gpio_set_dir(CLK_PIN, GPIO_OUT);
    
    // Set data pin as output
    gpio_init(DATA_PIN);
    gpio_set_dir(DATA_PIN, GPIO_OUT);

    // Set latch pin as output
    gpio_init(LATCH_PIN);
    gpio_set_dir(LATCH_PIN, GPIO_OUT);
    on_board_led_init();

    while(true){
        
        unsigned int x = 0b0000000000000000;   
        writeTo595(x);
        blink_led(1);
        
        int r = (rand() % 16); // 
        if (r == 0){
           writeTo595(ONE);
        }
        if(r == 1) {
           writeTo595(TWO);
        }
        if(r == 2) {
            writeTo595(THREE);
        }
        if(r == 3) {
            writeTo595(FOUR);
        }
        if(r == 4) {
            writeTo595(FIVE);
        }
        if(r == 5) {
            writeTo595(SIX);
        }
        if(r == 6) {
            writeTo595(SEVEN);
        }
        if(r == 7) {
            writeTo595(EIGHT);
        }
        if(r == 8) {
            writeTo595(NINE);
        }
        if(r == 9) {
            writeTo595(TEN);
        }
        if(r == 10) {
            writeTo595(ELEVEN);
        }
        if(r == 11) {
            writeTo595(TWELVE);
        }
        if(r == 12) {
            writeTo595(THIRTEEN);
        }
        if(r == 13) {
            writeTo595(FOURTEEN);
        }
        if(r == 14) {
            writeTo595(FIFTEEN);
        }
        if(r == 15) {
            writeTo595(SIXTEEN);
        }
        sleep_ms(600);
        
        x = 0b00000000000000000;       //0b 0000 0000
        blink_led(2);
        writeTo595(x);
        sleep_ms(1500);
    }
}
