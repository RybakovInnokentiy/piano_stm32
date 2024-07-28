#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <stdlib.h>
#include <stdio.h>
#include "main.h"

unsigned int volatile irq_timer_cnt = 0;
static struct piano_board piano_dev;

void uart_debug(char *str, int cnt){
    for(int i = 0; i < cnt; i++){
        usart_send_blocking(USART2, str[i]);

    }
    usart_send_blocking(USART2, '\r');
    usart_send_blocking(USART2, '\n');

}

void uart_debug_int(unsigned int number){
    char str_1[4];
    sprintf(str_1, "%02d", number);
    for(int i = 0; i < 4; i++){
        usart_send_blocking(USART2, str_1[i]);

    }
    usart_send_blocking(USART2, '\r');
    usart_send_blocking(USART2, '\n');

}

void tim2_isr(void) {
    timer_clear_flag(TIM2, TIM_SR_UIF);
    timer_disable_irq(TIM2, TIM_DIER_UIE);
    for(int i = 0; i < 8; i++){
        /*Write value to 74HC input*/
        GPIOB_ODR &= ~(0x07 << 3);
        GPIOB_ODR |= (i << 3);

        if(irq_timer_cnt % 2 == 0) {
            piano_dev.key_arr_1[i] =
                    ((GPIOD_IDR & GPIO2) >> 2) | ((GPIOC_IDR & GPIO12) >> 11) | ((GPIOC_IDR & GPIO11) >> 9) |
                    ((GPIOC_IDR & GPIO10) >> 7) | ((GPIOA_IDR & GPIO15) >> 11) | ((GPIOA_IDR & GPIO12) >> 7) |
                    ((GPIOA_IDR & GPIO11) >> 5) | ((GPIOA_IDR & GPIO10) >> 3) | ((GPIOA_IDR & GPIO9) >> 1) |
                    ((GPIOA_IDR & GPIO8) << 1) | ((GPIOC_IDR & GPIO9) << 1) | ((GPIOC_IDR & GPIO8) << 3) |
                    ((GPIOC_IDR & GPIO7) << 5) | ((GPIOC_IDR & GPIO6) << 7) | ((GPIOB_IDR & GPIO15) >> 1) |
                    ((GPIOB_IDR & GPIO14) << 1) | ((GPIOB_IDR & GPIO13) << 3) | ((GPIOB_IDR & GPIO12) << 5) |
                    ((GPIOB_IDR & GPIO10) << 8) | ((GPIOB_IDR & GPIO2) << 17) | ((GPIOB_IDR & GPIO1) << 19) |
                    ((GPIOB_IDR & GPIO0) << 21);
        }
        else {
            piano_dev.key_arr_2[i] =
                    ((GPIOD_IDR & GPIO2) >> 2) | ((GPIOC_IDR & GPIO12) >> 11) | ((GPIOC_IDR & GPIO11) >> 9) |
                    ((GPIOC_IDR & GPIO10) >> 7) | ((GPIOA_IDR & GPIO15) >> 11) | ((GPIOA_IDR & GPIO12) >> 7) |
                    ((GPIOA_IDR & GPIO11) >> 5) | ((GPIOA_IDR & GPIO10) >> 3) | ((GPIOA_IDR & GPIO9) >> 1) |
                    ((GPIOA_IDR & GPIO8) << 1) | ((GPIOC_IDR & GPIO9) << 1) | ((GPIOC_IDR & GPIO8) << 3) |
                    ((GPIOC_IDR & GPIO7) << 5) | ((GPIOC_IDR & GPIO6) << 7) | ((GPIOB_IDR & GPIO15) >> 1) |
                    ((GPIOB_IDR & GPIO14) << 1) | ((GPIOB_IDR & GPIO13) << 3) | ((GPIOB_IDR & GPIO12) << 5) |
                    ((GPIOB_IDR & GPIO10) << 8) | ((GPIOB_IDR & GPIO2) << 17) | ((GPIOB_IDR & GPIO1) << 19) |
                    ((GPIOB_IDR & GPIO0) << 21);
        }

        piano_dev.key_cnt_array[i]++;
        unsigned int arr_comparison = piano_dev.key_arr_1[i] ^ piano_dev.key_arr_2[i];
        if(arr_comparison){
            if((arr_comparison & CHEK_EVEN) && (piano_dev.key_event_array[i])) {
                piano_dev.key_cnt_array[i] = 0;
                piano_dev.key_event_array[i] = 0;
                usart_send_blocking(USART2, 'o');
                usart_send_blocking(USART2, 'k');
                usart_send_blocking(USART2, '\r');
                usart_send_blocking(USART2, '\n');

//                piano_dev.key_event_this->valid = 1;
//                piano_dev.key_event_this->press_time_2 = irq_timer_cnt;
//                piano_dev.key_event_this->note_state ^= 0x01;
//                piano_dev.key_event_this->button_num = arr_comparison;
//                piano_dev.key_event_this->button_row = i;
//                piano_dev.key_event_this = piano_dev.key_event_this->key_event_next;
            } else {
                if(piano_dev.key_cnt_array[i] > 10000){
                    piano_dev.key_cnt_array[i] = 0;
                    piano_dev.key_event_array[i] = 1;
                }
                //piano_dev.key_event_this->press_time_1 = irq_timer_cnt;
            }
        }
    }
    irq_timer_cnt++;
    timer_enable_irq(TIM2, TIM_DIER_UIE);
}

static void clock_setup(void) {
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_USART2);

}

void gpio_setup(void) {
    /*Setup GPIOB for working with matrix keyboard*/
    /*STM Output pins for 74HC24 Input decoder*/
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO3 | GPIO4 | GPIO5);

    /*STM Input pins from Piano shield output decoder*/
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN,
                    GPIO15 | GPIO14 | GPIO13 | GPIO12 | GPIO10 | GPIO2 | GPIO1 | GPIO0);
    gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO2);
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN,
                    GPIO12 | GPIO11 | GPIO10 | GPIO9 | GPIO8 | GPIO6 | GPIO7);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO15 | GPIO12 | GPIO11 | GPIO10 | GPIO9 | GPIO8);

    /*GPIO for leds*/
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);

}

void timer_setup(void) {
    timer_set_prescaler(TIM2, (rcc_apb1_frequency / 1000000) - 1); // 1MHz of TIM2
    timer_set_period(TIM2, 20 - 1); // Period = 20 us
    timer_enable_irq(TIM2, TIM_DIER_UIE);
    nvic_enable_irq(NVIC_TIM2_IRQ);
    timer_enable_counter(TIM2);
}

void usart_setup(void) {
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, 1);
    usart_set_mode(USART2, USART_MODE_TX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);
}

void piano_device_init(void) {
    for (int i = 0; i < 8; i++) {
        piano_dev.key_arr_1[i] = 0;
        piano_dev.key_arr_2[i] = 0;
        piano_dev.key_event_array[i] = 1;
        piano_dev.key_cnt_array[i] = 0;
        for(int j = 0; j < 22; j++) {
            piano_dev.key_cycles_array[i][j] = 0;
        }
    }

    static struct key_event key_event_arr[200] = {{0, 0,0, 0,0,0,NULL}};
    for(int i = 0; i < 199; i++){
        key_event_arr[i].key_event_next = &key_event_arr[i+1];
        key_event_arr[i].button_num = 0;
        key_event_arr[i].button_row = 0;
        key_event_arr[i].press_time_1 = 0;
        key_event_arr[i].press_time_2 = 0;
        key_event_arr[i].valid = 0;
        key_event_arr[i].note_state = 0;
    }
    key_event_arr[199].key_event_next = &key_event_arr[0];
    piano_dev.key_event_this = &key_event_arr[0];
    piano_dev.key_event_read_this = &key_event_arr[0];
}


void process_request(void) {
    unsigned int velocity = 0;
    unsigned int but_num = 0;
    if(piano_dev.key_event_read_this->valid != 0){
        if (piano_dev.key_event_read_this->note_state != 0){
            velocity = (6350) / (piano_dev.key_event_read_this->press_time_2 - piano_dev.key_event_read_this->press_time_1);
        }

        for(int j = 0; j < 22; j++){
            if(piano_dev.key_event_read_this->button_num & (1 << j)) {
                but_num = j;
                break;
            }
        }
        char str_2[10];
        sprintf(str_2, "%02d %02d %03d", piano_dev.key_event_read_this->note_state, piano_dev.key_event_read_this->button_row + but_num, velocity);
        piano_dev.key_event_read_this->valid = 0;
        for(int u = 0; u < 10; u++){
            usart_send_blocking(USART2, str_2[u]);
        }
        usart_send_blocking(USART2, '\r');
        usart_send_blocking(USART2, '\n');
        piano_dev.key_event_read_this = piano_dev.key_event_read_this->key_event_next;
    }

}

int main(void) {
    clock_setup();
    gpio_setup();
    usart_setup();
    piano_device_init();
    gpio_clear(GPIOA, GPIO7);
    timer_setup();
    while (1) {
//        uart_debug_int(irq_timer_cnt);
//        process_request();
    }
}
