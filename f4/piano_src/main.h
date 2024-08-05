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
#define NUM_OF_ELECTRODES     12

#define VOLUME_STEP           4

struct piano_board {
    unsigned int key_arr_1[8];
    unsigned int key_arr_2[8];
    unsigned int key_pedals_1;
    unsigned int key_pedals_2;
    unsigned int key_cycles_1[88];
    unsigned int key_event_push_array[88];
    unsigned int key_event_release_array[88];
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