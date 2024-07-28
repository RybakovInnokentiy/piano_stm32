#ifndef __PIANO_H_
#define __PIANO_H_

#define VELOCITY_STANDART_STEP 50
#define VELOCITY_CNT_MAX 127
#define CHEK_ODD      0x55555555
#define CHEK_EVEN      0xAAAAAAAA

struct piano_board {
    unsigned int key_arr_1[8];
    unsigned int key_arr_2[8];
    unsigned int key_cycles_2[8];
    unsigned int key_cycles_1[8];
    unsigned int key_cycles_array[8][22];
    unsigned int key_event_push_array[8];
    unsigned int key_event_release_array[8];
    unsigned int key_cnt_array[8];
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