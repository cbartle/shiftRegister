#include "pico/stdlib.h"

// #ifndef PICO_DEFAULT_LED_PIN
// #warning blink example requires a board with a regular LED
// #else
    static uint led_pin = 25;
    static bool state = false;


    void on_board_led_init_configured_state(bool on){
        state = on;
        gpio_init(led_pin);
        gpio_set_dir(led_pin, GPIO_OUT);
        gpio_put(led_pin, state);
    }
    
    void on_board_led_init(){
        on_board_led_init_configured_state(false);
    }

    void on_board_led_toggle(){
        state = !state;
        gpio_put(led_pin, state);
    }
//#endif