#pragma once
#include <esp32/ulp.h>
#include "soc/rtc_cntl_reg.h"

void ulp_light(gpio_num_t pin, int32_t pwm_bit, int32_t brightness, bool enable, bool& state) {
  if (enable) {
    if (!state) {
      gpio_reset_pin(pin);
      rtc_gpio_init(pin);
      rtc_gpio_set_direction(pin, RTC_GPIO_MODE_OUTPUT_ONLY);
      rtc_gpio_set_level(pin, 0);

      int32_t ulp_clock = 1000;

      int32_t on_time = map(brightness, 0, 255, 0, ulp_clock);
      int32_t off_time = ulp_clock - on_time;

      // Define ULP program
      const ulp_insn_t ulp_prog[] = {
        M_LABEL(1),
        I_WR_REG(RTC_GPIO_OUT_REG, pwm_bit, pwm_bit, 1), // on
        I_DELAY(on_time),
        I_WR_REG(RTC_GPIO_OUT_REG, pwm_bit, pwm_bit, 0), // off
        I_DELAY(off_time),
        M_BX(1)
      };

      // Run ULP program
      size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);
      ulp_process_macros_and_load(0, ulp_prog, &size);
      ulp_run(0);

      state = true;
    } 
  } else {
    if (state) {
      // Define ULP program
      const ulp_insn_t ulp_prog[] = {
        I_WR_REG(RTC_GPIO_OUT_REG, pwm_bit, pwm_bit, 0), // off
        I_HALT() 
      };

      // Run ULP program
      size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);
      ulp_process_macros_and_load(0, ulp_prog, &size);
      ulp_run(0);

      REG_CLR_BIT(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);

      gpio_reset_pin(pin);
      rtc_gpio_init(pin);
      rtc_gpio_set_direction(pin, RTC_GPIO_MODE_OUTPUT_ONLY);
      rtc_gpio_set_level(pin, 0);

      state = false;
    }
  }
}
