#ifndef __PIANO_H_
#define __PIANO_H_

#define VELOCITY_STEP 450

struct piano_board {
    unsigned int key_arr_1[8];
    unsigned int key_arr_2[8];
    unsigned int key_cycles_array[8][22];
    unsigned int key_event_array[8][22];
    struct key_event *key_event_this;
};

struct key_event {
    unsigned int button_num;
    unsigned int press_time_1;
    unsigned int press_time_2;
    unsigned int valid;
    unsigned int note_state; // 0 - off, 1 - on
    struct key_event *key_event_next;
};

#endif