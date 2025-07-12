#include "ads.h"

static const char *TAG = "ads";

esp_err_t ads_init(void) {
    const i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_22,
        .scl_io_num = GPIO_NUM_21,
        .master.clk_speed = 100000,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE
    };

    esp_err_t c =  i2c_param_config(I2C_NUM_0, &i2c_config);
    if (c != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed (%d)", c);
        return c;
    }
    static esp_err_t d = i2c_driver_install(I2C_NUM_0, i2c_config.mode, 0, 0, 0);
    if (d != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed (%d)", d);
        return d;
    }

    return ESP_OK;
}

esp_err_t config_error_check(const uint8_t channel, const uint8_t relative_to) {
    if (channel > CH3 || channel == relative_to || (relative_to != GND && relative_to != CH1 && relative_to != CH3)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (relative_to != GND) {
        if (relative_to == CH1 && channel != CH0) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    return ESP_OK;
}

uint8_t mux_value(const uint8_t channel, const uint8_t relative_to) {
    switch (relative_to) {
        case CH1: return MUX_P0_N1;
        case CH3: {
            switch (channel) {
                case CH0: return MUX_P0_N3;
                case CH1: return MUX_P1_N3;
                case CH2: return MUX_P2_N3;
                default: return MUX_P0_N3;
            }
        };
        case GND: {
            switch (channel) {
                case CH0: return MUX_P0_NGND;
                case CH1: return MUX_P1_NGND;
                case CH2: return MUX_P2_NGND;
                case CH3: return MUX_P3_NGND;
                default: return MUX_P0_NGND;
            }
        };
        default: return MUX_P0_N1;
    }
}

esp_err_t ads_config(const uint8_t channel, const uint8_t relative_to) {
    esp_err_t ret = config_error_check(channel, relative_to);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "config_error_check failed, wrong value provided for (%d)", ret);
        return ret;
    }

    const uint8_t mux_setup = mux_value(channel, relative_to) << 12;

    const uint16_t conf_word =  CONFIG_OS_READ << 15 |
                                mux_setup |
                                PGA_FSR_4_096 << 9 |
                                MODE_SINGLE_SHOT << 8 |
                                DR_128_SPS << 5 |
                                COMP_TRADITIONAL << 4 |
                                ACTIVE_LOW << 3 |
                                NON_LATCHING << 2 |
                                DISABLE_COMPARATOR;

    const uint8_t conf_word_msb = conf_word >> 8;
    const uint8_t conf_word_lsb = (conf_word & 0xFF);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "i2c_cmd_link_create failed");
        return ESP_FAIL;
    }
    i2c_master_start(cmd);

    ret = i2c_master_write_byte(cmd, (SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    if (ret != ESP_OK) goto err_finish;

    ret = i2c_master_write_byte(cmd, CONFIG_REGISTER, I2C_MASTER_ACK);
    if (ret != ESP_OK) goto err_finish;

    ret = i2c_master_write_byte(cmd, conf_word_msb, I2C_MASTER_ACK);
    if (ret != ESP_OK) goto err_finish;

    ret = i2c_master_write_byte(cmd, conf_word_lsb, I2C_MASTER_ACK);
    if (ret != ESP_OK) goto err_finish;

    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_begin failed %d", ret);
        i2c_cmd_link_delete(cmd);
        return ret;
    }

    i2c_cmd_link_delete(cmd);
    return ret;

err_finish:
    i2c_cmd_link_delete(cmd);
    ESP_LOGE(TAG, "I2C Error writing byte to a slave %d", ret);
    return ret;
}
