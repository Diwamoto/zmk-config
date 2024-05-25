/*
 * Copyright (c) 2024 diwamoto_
 *
 * SPDX-License-Identifier: MIT
 */


#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_kscan_ec_matrix

#define ROW_NUMS 5
#define COL_NUMS 6
#define MATRIX_CELLS (ROW_NUMS * COL_NUMS)
#define SEL_NUMS 3

#define WAIT_DISCHARGE()
#define WAIT_CHARGE()

#define COL_PINS {4, 6, 7, 5, 1, 0}

struct kscan_ec_matrix_config
{
    struct gpio_dt_spec rows[ROW_NUMS];
    struct gpio_dt_spec sels[SEL_NUMS];
    struct gpio_dt_spec discharge;
    struct gpio_dt_spec power;
    struct gpio_dt_spec mux_en;
    struct adc_dt_spec adc_channel;
    const uint8_t press_point;
    const uint8_t release_point;
    const uint8_t cols[COL_NUMS];
    const uint16_t matrix_relax_us;
    const uint16_t adc_read_settle_us;
    const uint16_t active_polling_interval_ms;
    const uint16_t idle_polling_interval_ms;
    const uint16_t sleep_polling_interval_ms;
};

struct kscan_ec_matrix_data
{
    kscan_callback_t callback;
    struct k_timer poll_timer;
    struct k_work poll;
    bool matrix_state[MATRIX_CELLS];
    const struct device *dev;
    struct adc_sequence adc_seq;
    int16_t adc_raw;
};

static int kscan_ec_matrix_configure(const struct device *dev, kscan_callback_t callback)
{
    LOG_DBG("KSCAN API configure");
    struct kscan_ec_matrix_data *data = dev->data;
    if (!callback)
    {
        return -EINVAL;
    }
    data->callback = callback;
    LOG_DBG("Configured KSCAN");
    return 0;
}

static int kscan_ec_matrix_enable(const struct device *dev)
{
    LOG_DBG("KSCAN API enable");
    struct kscan_ec_matrix_data *data = dev->data;
    const struct kscan_ec_matrix_config *cfg = dev->config;
    k_timer_start(&data->poll_timer,
                  K_MSEC(cfg->active_polling_interval_ms),
                  K_MSEC(cfg->active_polling_interval_ms));
    return 0;
}

static int kscan_ec_matrix_disable(const struct device *dev)
{
    LOG_DBG("KSCAN API disable");
    struct kscan_ec_matrix_data *data = dev->data;
    k_timer_stop(&data->poll_timer);
    return 0;
}

static void kscan_ec_matrix_timer_handler(struct k_timer *timer)
{
    struct kscan_ec_matrix_data *data =
        CONTAINER_OF(timer, struct kscan_ec_matrix_data, poll_timer);
    k_work_submit(&data->poll);
}

static void kscan_ec_matrix_work_handler(struct k_work *work)
{

    struct kscan_ec_matrix_data *data = CONTAINER_OF(work, struct kscan_ec_matrix_data, poll);
    const struct device *dev = data->dev;
    const struct kscan_ec_matrix_config *cfg = dev->config;
    struct adc_sequence *adc_seq = &data->adc_seq;
    // bool matrix_read[MATRIX_CELLS];

    gpio_pin_set_dt(&cfg->power, 1);

    for (int c = 0; c < COL_NUMS; ++c){ 
        for (int r = 0; r < ROW_NUMS; ++r) {

            gpio_pin_configure_dt(&cfg->discharge, GPIO_OUTPUT);

            // select mux sel
            uint8_t ch = cfg->cols[c];
            gpio_pin_set_dt(&cfg->sels[0], ch & 1);
            gpio_pin_set_dt(&cfg->sels[1], ch & 2);
            gpio_pin_set_dt(&cfg->sels[2], ch & 4);

            // clear all row pins
            for (int r2 = 0; r2 < ROW_NUMS; ++r2)
            {
                gpio_pin_set_dt(&cfg->rows[r2], 0);
            }

            const unsigned int lock = irq_lock();

            // charge capacitor
            gpio_pin_configure_dt(&cfg->discharge, GPIO_INPUT);
            gpio_pin_set_dt(&cfg->rows[r], 1);

            // read key 
            int rc = adc_read(cfg->adc_channel.dev, adc_seq);
            if (rc != 0) {
                LOG_ERR("Failed to read ADC: %d", rc);
                return;
            }

            int16_t sw_value = data->adc_raw;

            LOG_INF("row: %d, col: %d, value: %d", r, c, sw_value);
            irq_unlock(lock);

        }
    }

    gpio_pin_set_dt(&cfg->power, 0);
}

static int kscan_ec_matrix_activity_event_handler(const struct device *dev, const zmk_event_t *eh)
{
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev == NULL)
    {
        return -ENOTSUP;
    }
    struct kscan_ec_matrix_data *data = dev->data;
    const struct kscan_ec_matrix_config *cfg = dev->config;
    uint16_t poll_interval;
    switch (ev->state)
    {
    case ZMK_ACTIVITY_ACTIVE:
        poll_interval = cfg->active_polling_interval_ms;
        break;
    case ZMK_ACTIVITY_IDLE:
        poll_interval = cfg->idle_polling_interval_ms;
        break;
    case ZMK_ACTIVITY_SLEEP:
        poll_interval = cfg->sleep_polling_interval_ms;
        break;
    default:
        LOG_WRN("Unhandled activity state: %d", ev->state);
        return -EINVAL;
    }
    LOG_DBG("Setting poll interval to %d", poll_interval);
    k_timer_start(&data->poll_timer, K_MSEC(10), K_MSEC(10));
    return 0;
}

static int kscan_ec_matrix_init(const struct device *dev)
{
    LOG_INF("KSCAN init");
    struct kscan_ec_matrix_data *data = dev->data;
    const struct kscan_ec_matrix_config *cfg = dev->config;
    data->dev = dev;

    // enable mux
    gpio_pin_configure_dt(&cfg->mux_en, GPIO_OUTPUT_INACTIVE);

    // power on
    gpio_pin_configure_dt(&cfg->power, GPIO_OUTPUT_ACTIVE);

    // discharge mode
    gpio_pin_configure_dt(&cfg->discharge, GPIO_OUTPUT_INACTIVE);

    int rc = 0;

    rc = adc_channel_setup_dt(&cfg->adc_channel);
    if (rc < 0) {
        LOG_ERR("ADC channel setup error %d", rc);
    }

    data->adc_seq = (struct adc_sequence){
      .buffer = &data->adc_raw,
      .buffer_size = sizeof(data->adc_raw),
    };

    rc = adc_sequence_init_dt(&cfg->adc_channel, &data->adc_seq);
    if (rc < 0) {
        LOG_ERR("ADC sequence init error %d", rc);
    }

    // init rows
    for (int i = 0; i < ROW_NUMS; ++i)
    {
        gpio_pin_configure_dt(&cfg->rows[i], GPIO_OUTPUT_INACTIVE);
    }

    // init mux sels
    for (int i = 0; i < SEL_NUMS; ++i)
    {
        gpio_pin_configure_dt(&cfg->sels[i], GPIO_OUTPUT);
    }

    k_timer_init(&data->poll_timer, kscan_ec_matrix_timer_handler, NULL);
    k_work_init(&data->poll, kscan_ec_matrix_work_handler);

    return 0;
}
static const struct kscan_driver_api kscan_ec_matrix_api = {
    .config = kscan_ec_matrix_configure,
    .enable_callback = kscan_ec_matrix_enable,
    .disable_callback = kscan_ec_matrix_disable,
};

#define CREATE_KSCAN_EC_MATRIX(inst)                                                         \
    static struct kscan_ec_matrix_data kscan_ec_matrix_data##inst;                          \
    static const struct kscan_ec_matrix_config kscan_ec_matrix_config##inst = {               \
        .rows =                                                                             \
            {                                                                               \
                GPIO_DT_SPEC_INST_GET_BY_IDX(inst, row_gpios, 0),                           \
                GPIO_DT_SPEC_INST_GET_BY_IDX(inst, row_gpios, 1),                           \
            },                                                                              \
        .cols = COL_PINS,                                                                   \
        .sels =                                                                             \
            {                                                                               \
                GPIO_DT_SPEC_INST_GET_BY_IDX(inst, sel_gpios, 0),                           \
                GPIO_DT_SPEC_INST_GET_BY_IDX(inst, sel_gpios, 1),                           \
                GPIO_DT_SPEC_INST_GET_BY_IDX(inst, sel_gpios, 2),                           \
            },                                                                              \
        .discharge = GPIO_DT_SPEC_INST_GET_BY_IDX(inst, discharge_gpios, 0),                \
        .power = GPIO_DT_SPEC_INST_GET_BY_IDX(inst, power_gpios, 0),                        \
        .mux_en = GPIO_DT_SPEC_INST_GET_BY_IDX(inst, mux_en_gpios, 0),                      \
        .adc_channel = ADC_DT_SPEC_INST_GET(inst),                                          \
        .press_point = DT_INST_PROP(inst, press_point),                                     \
        .release_point = DT_INST_PROP(inst, release_point),                                 \
        .active_polling_interval_ms = DT_INST_PROP(inst, active_polling_interval_ms),       \
        .idle_polling_interval_ms = DT_INST_PROP(inst, idle_polling_interval_ms),           \
        .sleep_polling_interval_ms = DT_INST_PROP(inst, sleep_polling_interval_ms),         \
    };                                                                                      \
    static int kscan_ec_matrix_activity_event_handler_wrapper##inst(const zmk_event_t *eh)  \
    {                                                                                       \
        const struct device *dev = DEVICE_DT_INST_GET(inst);                                \
        return kscan_ec_matrix_activity_event_handler(dev, eh);                             \
    }                                                                                       \
    ZMK_LISTENER(kscan_ec_matrix##inst,                                                     \
                 kscan_ec_matrix_activity_event_handler_wrapper##inst);                     \
    ZMK_SUBSCRIPTION(kscan_ec_matrix##inst, zmk_activity_state_changed);                    \
    DEVICE_DT_INST_DEFINE(inst,                                                             \
                          kscan_ec_matrix_init,                                             \
                          NULL,                                                             \
                          &kscan_ec_matrix_data##inst,                                      \
                          &kscan_ec_matrix_config##inst,                                     \
                          APPLICATION,                                                      \
                          CONFIG_APPLICATION_INIT_PRIORITY,                                 \
                          &kscan_ec_matrix_api);

DT_INST_FOREACH_STATUS_OKAY(CREATE_KSCAN_EC_MATRIX)
