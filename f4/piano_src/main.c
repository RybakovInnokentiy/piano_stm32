#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <stdio.h>

void tim2_isr(void) {
    timer_clear_flag(TIM2, TIM_SR_UIF);
    gpio_toggle(GPIOA, GPIO7);
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
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO15 | GPIO14 | GPIO13 | GPIO12 | GPIO10 | GPIO2 | GPIO1 | GPIO0);
    gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO2);
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO12 | GPIO11 | GPIO10 | GPIO9 | GPIO8 | GPIO6 | GPIO7);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO15 | GPIO12 | GPIO11 | GPIO10 | GPIO9 | GPIO8);

    /*GPIO for leds*/
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);

}

void timer_setup(void) {
    timer_set_prescaler(TIM2, (rcc_apb1_frequency / 1000000) - 1); // 1MHz of TIM2
    timer_set_period(TIM2, 20-1); // Period = 20 us
    timer_enable_irq(TIM2, TIM_DIER_UIE);
    nvic_enable_irq(NVIC_TIM2_IRQ);
    timer_enable_counter(TIM2);
}

void usart_setup(void) {

}

int main(void) {
    clock_setup();
    gpio_setup();
    timer_setup();
    usart_setup();
//    gpio_clear(GPIOA, GPIO7);
    gpio_set(GPIOB, GPIO3);

    while (1) {

    }
}