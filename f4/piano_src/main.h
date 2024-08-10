#ifndef __PIANO_H_
#define __PIANO_H_

#define VELOCITY_STANDART_STEP 50
#define VELOCITY_CNT_MAX 127
#define CHEK_ODD      0x55555555
#define CHEK_EVEN      0xAAAAAAAA
#define CHANNEL_PEDAL_LEFT   64
#define CHANNEL_PEDAL_MIDDLE   66
#define CHANNEL_PEDAL_RIGHT   67
#define MPR121_ADDR           0x5B
#define EEPROM_ADDR           0x50 // 0xA0 >> 1

#define NUM_OF_ELECTRODES     12
#define VOLUME_ADDRESS        0x00
#define VOLUME_STEP           4

//for colors
#define led_B1              200
#define led_W1              20
#define led_R1              0
#define led_G1              0

#define led_B2              310     //Must not exeed B1 + (PWM_LED_PERIOD - B1) / 127 !
#define led_W2              60     //Must not exeed W1 + (PWM_LED_PERIOD - W1) / 127 !
#define led_R2              0       //Must not exeed R1 + (PWM_LED_PERIOD - R1) / 127 !
#define led_G2              0       //Must not exeed G1 + (PWM_LED_PERIOD - G1) / 127 !

#define PWM_LED_PERIOD      15000

struct piano_board {
    unsigned int key_arr_1[8];
    unsigned int key_arr_2[8];
    unsigned int key_pedals_1;
    unsigned int key_pedals_2;
    unsigned int key_cycles_1[120];
    unsigned int key_event_push_array[120];
    unsigned int key_event_release_array[120];
    unsigned int key_cnt_array[8];
    unsigned int key_buttons_array[8];
    struct key_event *key_event_this;
    struct key_event *key_event_read_this;
};

struct key_event {
    unsigned int button_num;
    unsigned int button_row;
    unsigned int press_time_1;
    unsigned int press_time_2;
    unsigned int valid;
    unsigned int note_state; // 0 - off, 1 - on
    struct key_event *key_event_next;
};

#endif