#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


static void clock_setup(void) {
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
}

int main(void) {
    clock_setup();
    
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);
    while (1) {
        for (int i = 0; i < 1000000; i++) {
            __asm__("nop");
        }
        gpio_toggle(GPIOA, GPIO7);
    }
}