/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Espressif Systems
 *    integrated circuit in a product or a software update for such product,
 *    must reproduce the above copyright notice, this list of conditions and
 *    the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * 4. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "switch_driver.h"

#define LD2410_OUTPUT_GPIO GPIO_NUM_12

static QueueHandle_t gpio_evt_queue = NULL;
/* call back function pointer */
static esp_switch_callback_t func_ptr;

static const char *TAG = "ESP_ZB_LD2410";
static const IRAM_ATTR int ld2410_output_gpio = LD2410_OUTPUT_GPIO;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    xQueueSendFromISR(gpio_evt_queue, (int*)arg, NULL);
}

/**
 * @brief Enable GPIO isr
 *
 * @param enabled      enable isr if true.
 */
static void switch_driver_gpios_intr_enabled(bool enabled)
{
    if (enabled) {
        gpio_intr_enable(LD2410_OUTPUT_GPIO);
    } else {
        gpio_intr_disable(LD2410_OUTPUT_GPIO);
    }
}

/**
 * @brief Tasks for checking the button event and debounce the switch state
 *
 * @param arg      Unused value.
 */
static void switch_driver_button_detected(void *arg)
{
    int pin;
    bool first_run = true;

    const TickType_t periodic_timeout_ticks = pdMS_TO_TICKS(5000);

    for (;;) {
        if (first_run || xQueueReceive(gpio_evt_queue, &pin, periodic_timeout_ticks)) {
            switch_driver_gpios_intr_enabled(false);

            bool value = gpio_get_level(pin);
            ESP_LOGD(TAG, "queue, pin: %d, level: %d", pin, value);

            (*func_ptr)(&value);

            switch_driver_gpios_intr_enabled(true);
        } else {
            switch_driver_gpios_intr_enabled(false);

            bool value = gpio_get_level(LD2410_OUTPUT_GPIO);
            ESP_LOGD(TAG, "pin: %d, level: %d", LD2410_OUTPUT_GPIO, value);

            (*func_ptr)(&value);

            switch_driver_gpios_intr_enabled(true);
        }

        first_run = false;
    }
}

/**
 * @brief init GPIO configuration as well as isr
 *
 * @param button_func_pair      pointer of the button pair.
 * @param button_num            number of button pair.
 */
static bool switch_driver_gpio_init()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LD2410_OUTPUT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };

    gpio_config(&io_conf);

    /* create a queue to handle gpio event from isr */
    gpio_evt_queue = xQueueCreate(10, sizeof(bool));
    if ( gpio_evt_queue == 0) {
        ESP_LOGE(TAG, "Queue was not created and must not be used");

        return false;
    }

    /* install gpio isr service */
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(LD2410_OUTPUT_GPIO, gpio_isr_handler, (void *) &ld2410_output_gpio);
    /* start gpio task */
    xTaskCreate(switch_driver_button_detected, "button_detected", 6144, NULL, 10, NULL);

    switch_driver_gpios_intr_enabled(true);

    return true;
}

bool switch_driver_init(esp_switch_callback_t cb)
{
    func_ptr = cb;

    return switch_driver_gpio_init();
}
