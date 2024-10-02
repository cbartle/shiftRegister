#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <time.h>
#include <math.h>
//#include "noaa-data.c"
#include "i2c-display-lib.h"
#include "mpl3115a2.h"

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

uint16_t tempLights[] = {
    0b0000000000000001,
    0b0000000000000011,
    0b0000000000000111,
    0b0000000000001111,
    0b0000000000011111,
    0b0000000000111111,
    0b0000000001111111,
    0b0000000011111111,
    0b0000000111111111,
    0b0000001111111111,
    0b0000011111111111,
    0b0000111111111111,
    0b0001111111111111,
    0b0011111111111111,
    0b0111111111111111,
    0b1111111111111111
};

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

void reset_leds(){
    writeTo595(0b00000000000000000);
}

void process_farenheit(float temp_f){
    reset_leds();
    lcd_clear();
    char str[16];

    double rounded = round(temp_f);
    sprintf(str, "%.2f", rounded);
    lcd_home();
    char temp[] = "Temp: ";
    strcat(temp, str);
    strcat(temp, " F");
    lcd_print(temp);
    double result = rounded / 6;
    int resultInt = (int)result;
    writeTo595(tempLights[resultInt]);

    return;
}

void process_pressure(float pressure){
    //pressure is in pascals.

    float lowest_recorded = 87000.00;
    float highest_recorded = 108380.00;
    float atmosphericPressure = pressure * 0.00986923;
    
    char str[16];
    double rounded = round(atmosphericPressure);
    sprintf(str, "%.4f", rounded);
    lcd_setCursor(1,0);
    lcd_print("                ");
    char pres[] = "ATM: ";
    strcat(pres, str);
    strcat(pres, " atm");
    lcd_setCursor(1,0);
    lcd_print(pres);
    // reset_leds();

    // int pressure_led = -1;
    // for(float i = lowest_recorded; i < highest_recorded; i = i+2120){
    //     pressure_led++;        
    // } 
    // if(pressure_led > -1){
    //     writeTo595(tempLights[pressure_led]);
    // }
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
    //on_board_led_init();

    lcd_init(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN); // sda and scl

    lcd_home(); // or lcd_setCursor(0,0);
    //lcd_print("Hello World!");
    lcd_print("a");
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(500);

    lcd_clear();
    lcd_print("Initializing....");
    sensor_init(true); 

    reset_leds();
    while(true){
        
        if (has_new_data) {

            lcd_clear();
            lcd_home();
            lcd_print("Getting new data");
            read_data();

            time_t rawtime;
            struct tm * timeinfo;

            time ( &rawtime );
            timeinfo = localtime ( &rawtime );
            printf ( "Current local time and date: %s", asctime (timeinfo) );
            float farenheit = get_farenheit_temp()/* i*/;
            float celcius = get_celcius_temp();
            float altitude = get_altitude();
            float pressure = get_pressure();

            process_farenheit(farenheit);
            process_pressure(pressure);
            printf("%d sample average -> Farenheit: %.4f F, Celsius: %.4f, h: %.4f m, Pressure: %.4f kPa\n", MPL3115A2_FIFO_SIZE, farenheit, celcius, altitude, pressure);           
            
        }
    }
}
