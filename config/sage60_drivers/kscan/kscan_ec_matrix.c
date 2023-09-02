/*
 * Copyright (c) 2022, 2023 diwamoto_
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_kscan_ec_matrix

#include <zephyr/device.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define ROW_NUMS 5
#define COL_NUMS 6
#define MATRIX_CELLS (ROW_NUMS * COL_NUMS)
#define SEL_NUMS 3

#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_6

#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7
#define S8 8
#define COL_PINS {S7, S5, S3, S8, S1, S2}

struct kscan_ec_matrix_config
{
    struct gpio_dt_spec rows[ROW_NUMS];
    struct gpio_dt_spec sels[SEL_NUMS];
    struct gpio_dt_spec discharge;
    const uint8_t adc_channel;
    const uint8_t press_point;
    const uint8_t release_point;
    const uint8_t cols[COL_NUMS];
    const uint16_t matrix_relax_us;
    const uint16_t adc_read_settle_us;
    const uint16_t active_polling_interval_ms;
    const uint16_t idle_polling_interval_ms;
    const uint16_t sleep_polling_interval_ms;
};

struct analog_capacitor_value {
    uint16_t adc_raw;
    uint16_t millivolts;
    uint8_t state_of_charge;
};


struct kscan_ec_matrix_data
{
    kscan_callback_t callback;
    struct k_timer poll_timer;
    struct k_work poll;
    bool matrix_state[MATRIX_CELLS];
    struct adc_sequence as;
    struct analog_capacitor_value value;
    const struct device *dev;
    const struct device *adc;
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
    const struct device *adc = data->adc;
    const struct kscan_ec_matrix_config *cfg = dev->config;
    bool matrix_read[MATRIX_CELLS];

    for (int r = 0; r < ROW_NUMS; ++r)
    {
        for (int c = 0; c < COL_NUMS; ++c)
        {

            // discharge capacitor
            gpio_pin_configure(cfg->discharge.port,
                            cfg->discharge.pin,
                            GPIO_OUTPUT_INACTIVE | cfg->discharge.dt_flags);
            k_sleep(K_MSEC(10));
            
            // select mux sel
            uint8_t ch = cfg->cols[c];
            gpio_pin_set(cfg->sels[0].port, cfg->sels[0].pin, ch & 1);
            gpio_pin_set(cfg->sels[1].port, cfg->sels[1].pin, ch & 2);
            gpio_pin_set(cfg->sels[2].port, cfg->sels[2].pin, ch & 4);
            k_sleep(K_MSEC(10));

            // clear all row pins 
            for (int r2 = 0; r2 < ROW_NUMS; ++r2)
            {
                gpio_pin_set(cfg->rows[r2].port, cfg->rows[r2].pin, 0);
            }
            k_sleep(K_MSEC(10));
            
            // charge capacitor
            gpio_pin_configure(cfg->discharge.port,
                    cfg->discharge.pin,
                    GPIO_INPUT | cfg->discharge.dt_flags);
            gpio_pin_set(cfg->rows[r].port, cfg->rows[r].pin, 0);
            k_sleep(K_MSEC(10));

            // read analog value
            struct adc_sequence *as = &data->as;
            int sw_value = adc_read(adc, as);
            as->calibrate = false;
            
            int cell = (r * COL_NUMS) + c;

            if (sw_value >= cfg->press_point) {
                matrix_read[cell] = true;
            } else if (sw_value <= cfg->release_point) {
                matrix_read[cell] = false;
            }

            // // これじゃだめなのかしら
            // if (data->matrix_state[cell] != matrix_read[cell])
            // {
            //     data->matrix_state[cell] = matrix_read[cell];
            //     data->callback(data->dev, r, c, matrix_read[cell]);
            // }
        }
    }

    // TODO: 二回回す意味ある？
    for (int r = 0; r < ROW_NUMS; ++r)
    {
        for (int c = 0; c < COL_NUMS; ++c)
        {
            int cell = (r * COL_NUMS) + c;
            if (data->matrix_state[cell] != matrix_read[cell])
            {
                data->matrix_state[cell] = matrix_read[cell];
                data->callback(data->dev, r, c, matrix_read[cell]);
            }
        }
    }
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
    k_timer_start(&data->poll_timer, K_MSEC(poll_interval), K_MSEC(poll_interval));
    return 0;
}

static int kscan_ec_matrix_init(const struct device *dev)
{
    LOG_DBG("KSCAN init");
    struct kscan_ec_matrix_data *data = dev->data;
    const struct kscan_ec_matrix_config *cfg = dev->config;
    data->dev = dev;

    // get adc device
    data->adc = DEVICE_DT_GET(DT_NODELABEL(adc));

    // discharge mode
    gpio_pin_configure(cfg->discharge.port,
                    cfg->discharge.pin,
                    GPIO_OUTPUT_INACTIVE | cfg->discharge.dt_flags);
    gpio_pin_set(cfg->discharge.port, cfg->discharge.pin, 0);

    // init rows
    for (int i = 0; i < ROW_NUMS; ++i)
    {
        gpio_pin_configure(cfg->rows[i].port,
                           cfg->rows[i].pin,
                           GPIO_OUTPUT_INACTIVE | cfg->rows[i].dt_flags);
        gpio_pin_set(cfg->rows[i].port, cfg->rows[i].pin, 0);
    }

    // init mux sel pins
    for (int i = 0; i < SEL_NUMS; ++i)
    {
        gpio_pin_configure(cfg->sels[i].port,
                           cfg->sels[i].pin,
                           GPIO_OUTPUT_INACTIVE | cfg->sels[i].dt_flags);
    }

    gpio_pin_configure(cfg->discharge.port,
                    cfg->discharge.pin,
                    GPIO_INPUT | cfg->discharge.dt_flags);

    k_timer_init(&data->poll_timer, kscan_ec_matrix_timer_handler, NULL);
    k_work_init(&data->poll, kscan_ec_matrix_work_handler);

    // init adc sequence
    data->as = (struct adc_sequence){
        .channels = BIT(cfg->adc_channel),
        .buffer = &data->value.adc_raw,
        .buffer_size = sizeof(data->value.adc_raw),
        .oversampling = 4,
        .calibrate = true,
    };

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
                GPIO_DT_SPEC_INST_GET_BY_IDX(inst, gpios, 0),                               \
                GPIO_DT_SPEC_INST_GET_BY_IDX(inst, gpios, 1),                               \
                GPIO_DT_SPEC_INST_GET_BY_IDX(inst, gpios, 2),                               \
                GPIO_DT_SPEC_INST_GET_BY_IDX(inst, gpios, 3),                               \
                GPIO_DT_SPEC_INST_GET_BY_IDX(inst, gpios, 4),                               \
            },                                                                              \
        .cols = COL_PINS,                                                                   \
        .sels =                                                                             \
            {                                                                               \
                GPIO_DT_SPEC_INST_GET_BY_IDX(inst, gpios, 5),                               \
                GPIO_DT_SPEC_INST_GET_BY_IDX(inst, gpios, 6),                               \
                GPIO_DT_SPEC_INST_GET_BY_IDX(inst, gpios, 7),                               \
            },                                                                              \
        .discharge = GPIO_DT_SPEC_INST_GET_BY_IDX(inst, gpios, 8),                          \
        .adc_channel = DT_INST_PROP(inst, adc_channel),                                     \
        .press_point = DT_INST_PROP(inst, press_point),                                     \
        .release_point = DT_INST_PROP(inst, release_point),                                 \
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
