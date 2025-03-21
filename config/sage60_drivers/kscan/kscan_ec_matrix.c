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

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_kscan_ec_matrix
#define SAMPLE_INTERVAL K_MSEC(10)

struct kscan_ec_matrix_config
{
    const struct gpio_dt_spec *rows;
    const struct gpio_dt_spec *sels;
    struct gpio_dt_spec discharge;
    struct gpio_dt_spec power;
    struct gpio_dt_spec mux_en;
    struct adc_dt_spec adc_channel;
    const uint16_t press_point;
    const uint16_t release_point;
    const uint8_t *cols;
    uint8_t row_num;
    uint8_t col_num;
    uint8_t sel_num;
};

struct kscan_ec_matrix_data
{
    struct k_work_delayable work;
    kscan_callback_t callback;
    bool matrix_state[32][32];
    const struct device *dev;
    uint16_t adc_raw;
    struct adc_sequence adc_seq;
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
    return k_work_schedule(&data->work, K_NO_WAIT);
}

static int kscan_ec_matrix_disable(const struct device *dev)
{
    LOG_DBG("KSCAN API disable");
    struct kscan_ec_matrix_data *data = dev->data;
    return k_work_cancel_delayable(&data->work);
}

static void kscan_ec_matrix_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct kscan_ec_matrix_data *data = CONTAINER_OF(dwork, struct kscan_ec_matrix_data, work);
    const struct device *dev = data->dev;
    const struct kscan_ec_matrix_config *cfg = dev->config;

    for (int c = 0; c < cfg->col_num; ++c){

        for (int r = 0; r < cfg->row_num; ++r) {

            // select mux sel
            uint8_t ch = cfg->cols[c];
            gpio_pin_set_dt(&cfg->sels[0], ch & 1);
            gpio_pin_set_dt(&cfg->sels[1], (ch & 2) >> 1);
            gpio_pin_set_dt(&cfg->sels[2], (ch & 4) >> 2);
            
            // clear all row pins
            for (int r2 = 0; r2 < cfg->row_num; ++r2)
            {
                gpio_pin_set_dt(&cfg->rows[r2], 0);
            }

            // charge capacitor
            gpio_pin_configure_dt(&cfg->discharge, GPIO_INPUT);
            gpio_pin_set_dt(&cfg->rows[r], 1);

            // read key 
            int err = adc_read(cfg->adc_channel.dev, &data->adc_seq);
            if (err != 0) {
                LOG_ERR("Failed to read ADC: %d", err);
                return;
            }

            // LOG_DBG("Row %d, Col %d, ADC raw value: %d", r, c, data->adc_raw);

            // bool old_state = data->matrix_state[r][c];
            if (data->adc_raw > 1000) {
                LOG_INF("row: %d, col: %d val: %d", r, c, data->adc_raw);
            }
            // if (data->adc_raw > cfg->press_point && !old_state)
            // {
            //     LOG_INF("Key press detected at [%d,%d] value=%d", r, c, data->adc_raw);
            //     data->matrix_state[r][c] = true;
            //     if (data->callback)
            //     {
            //         // data->callback(dev, r, c, data->matrix_state[r][c]);
            //     }
            // }
            // else if (data->adc_raw < cfg->release_point && old_state)
            // {
            //     data->matrix_state[r][c] = false;
            //     if (data->callback)
            //     {
            //         // data->callback(dev, r, c, data->matrix_state[r][c]);
            //     }
            // }
        }
    }

    k_work_schedule(&data->work, SAMPLE_INTERVAL);
}

static int kscan_ec_matrix_init(const struct device *dev)
{
    LOG_INF("KSCAN init");
    struct kscan_ec_matrix_data *data = dev->data;
    const struct kscan_ec_matrix_config *cfg = dev->config;
    data->dev = dev;

    // enable mux
    int ret = gpio_pin_configure_dt(&cfg->mux_en, GPIO_OUTPUT);
    if (ret < 0) return ret;
    gpio_pin_set_dt(&cfg->mux_en, 0);

    // power on
    ret = gpio_pin_configure_dt(&cfg->power, GPIO_OUTPUT);
    if (ret < 0) return ret;
    gpio_pin_set_dt(&cfg->power, 1);

    // discharge mode
    ret = gpio_pin_configure_dt(&cfg->discharge, GPIO_OUTPUT);
    if (ret < 0) return ret;
    gpio_pin_set_dt(&cfg->discharge, 0);

    // init rows
    for (int i = 0; i < cfg->row_num; i++)
    {
        ret = gpio_pin_configure_dt(&cfg->rows[i], GPIO_OUTPUT);
        if (ret < 0) return ret;
        gpio_pin_set_dt(&cfg->rows[i], 0);
    }

    // init mux sels
    for (int i = 0; i < cfg->sel_num; ++i)
    {
        ret = gpio_pin_configure_dt(&cfg->sels[i], GPIO_OUTPUT);
        if (ret < 0) return ret;
        gpio_pin_set_dt(&cfg->sels[i], 0);
    }

    // Configure ADC
    if (!device_is_ready(cfg->adc_channel.dev)) {
        return -ENODEV;
    }

    data->adc_seq = (struct adc_sequence){
        .channels = BIT(cfg->adc_channel.channel_id),
        .buffer = &data->adc_raw,
        .buffer_size = sizeof(data->adc_raw),
        .resolution = cfg->adc_channel.resolution,
        .calibrate = true,
    };
     
    ret = adc_channel_setup_dt(&cfg->adc_channel);
    if (ret < 0) {
        LOG_ERR("ADC channel setup error %d", ret);
    }

    k_work_init_delayable(&data->work, kscan_ec_matrix_work_handler);

    return 0;
}

static const struct kscan_driver_api kscan_ec_matrix_api = {
    .config = kscan_ec_matrix_configure,
    .enable_callback = kscan_ec_matrix_enable,
    .disable_callback = kscan_ec_matrix_disable,
};

#define KSCAN_EC_MATRIX_INIT(inst)                                                           \
    static const struct gpio_dt_spec row_pins_##inst[] = {                                  \
        GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst), row_gpios, 0),                           \
        GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst), row_gpios, 1),                           \
        GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst), row_gpios, 2),                           \
        GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst), row_gpios, 3),                           \
        GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst), row_gpios, 4),                           \
    };                                                                                      \
    static const struct gpio_dt_spec sel_pins_##inst[] = {                                  \
        GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst), sel_gpios, 0),                           \
        GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst), sel_gpios, 1),                           \
        GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst), sel_gpios, 2),                           \
    };                                                                                      \
    static const uint8_t col_channels_##inst[] = DT_INST_PROP(inst, col_channels);          \
    static const struct kscan_ec_matrix_config config_##inst = {                              \
        .rows = row_pins_##inst,                                                            \
        .sels = sel_pins_##inst,                                                            \
        .cols = col_channels_##inst,                                                        \
        .discharge = GPIO_DT_SPEC_GET(DT_DRV_INST(inst), discharge_gpios),                  \
        .power = GPIO_DT_SPEC_GET(DT_DRV_INST(inst), power_gpios),                          \
        .mux_en = GPIO_DT_SPEC_GET(DT_DRV_INST(inst), mux_en_gpios),                        \
        .adc_channel = ADC_DT_SPEC_INST_GET(inst),                                          \
        .row_num = ARRAY_SIZE(row_pins_##inst),                                             \
        .col_num = ARRAY_SIZE(col_channels_##inst),                                         \
        .sel_num = ARRAY_SIZE(sel_pins_##inst),                                             \
        .press_point = DT_INST_PROP(inst, press_point),                                     \
        .release_point = DT_INST_PROP(inst, release_point),                                 \
    };                                                                                      \
    static struct kscan_ec_matrix_data data_##inst;                                         \
    DEVICE_DT_INST_DEFINE(inst,                                                             \
                          &kscan_ec_matrix_init,                                             \
                          NULL,                                                             \
                          &data_##inst,                                                     \
                          &config_##inst,                                                    \
                          POST_KERNEL,                                                      \
                          CONFIG_KSCAN_INIT_PRIORITY,                                       \
                          &kscan_ec_matrix_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_EC_MATRIX_INIT)
