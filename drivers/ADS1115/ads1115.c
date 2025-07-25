/*
 * Filename: ads.c
 * Author: Paweł Plewa
 * Description: A driver for an easy usage of ADS111x made for ESP32 series.
 * License: MIT
 * LinkedIn: www.linkedin.com/in/paweł-plewa-78b398250
 */

#include "ads.h"

// ESP logs tag
static const char *TAG = "ads";

esp_err_t ads_init(void) {
    // bool value preventing user from
    // trying to install the same i2c
    // drivers on ESP32 board twice
    static bool ads_initialised = false;
    if (ads_initialised) {
        ESP_LOGI(TAG, "ADS already initialised.");
        return ESP_OK;
    }

    // configuration of I2C interface
    const i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER, // our ESP32 is in a master mode
        .sda_io_num = GPIO_NUM_22,
        .scl_io_num = GPIO_NUM_21,
        .master.clk_speed = 100000, // clock speed 100kHz
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE
    };

    const esp_err_t c =  i2c_param_config(I2C_NUM_0, &i2c_config);
    if (c != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed (%d)", c);
        return c;
    }
    // after configuration, installing an I2C driver
    const esp_err_t d = i2c_driver_install(I2C_NUM_0, i2c_config.mode, 0, 0, 0);
    if (d != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed (%d)", d);
        return d;
    }

    ads_initialised = true;

    return ESP_OK;
}

/*
 * ADS1115 has 8 possible multiplexer configurations.
 * This function makes sure that a user isn't trying to
 * set up a wrong one. ADS1113 and ADS1114 only have one
 * default configuration each.
 */
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

/*
 * The purpose of this function is to properly set
 * the input multiplexer of ADS1115. The function
 * returns the proper value of bits 14:12 of the
 * Config Register based on the desired inputs
 * of the user. This function must be called
 * after the config_error_check function to make
 * sure that the configuration exists. The mux_value
 * function itself does not check for errors.
 */
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
        default: return MUX_P0_N1; // a default value for all ADS111x
    }
}

/*
 * This function sets the bits 11:9 of the
 * Config Register, which set the full-scale range
 * (FSR) of a programmable gain amplifier (PGA).
 * The function returns the right bit sequence
 * based on the user-provided max voltage input.
 * Those bits serve no function on ADS1113.
 */
uint8_t pga_value(const double vdd) {
    // there are 6 possible PGA setups
    const uint8_t pga_options_quantity = 6;
    // variable for storing the right FSR
    double pga_range_found = 0;
    const double maximum_rating = vdd + 0.3;

    // 6 possible FSRs
    const double pga_available[] = {0.256, .512, 1.024, 2.048, 4.096, 6.144};
    for (int i = 0; i < pga_options_quantity; i++) {
        // we iterate until we find a value larger than the maximum rating
        if (maximum_rating < pga_available[i]) {
            pga_range_found = pga_available[i];
            break;
        }
    }
    if (pga_range_found == 0) {
        ESP_LOGI(TAG, "No proper PGA range was found, defaulting to 2.048V");
    }

    // ESP_LOGI("PGA", "FSR %d", (int)pga_range_found);

    // then we match the found range with a proper bit sequence.
    if (pga_range_found == .256) return PGA_FSR_0_256;
    else if (pga_range_found == .512) return PGA_FSR_0_512;
    else if (pga_range_found == 1.024) return PGA_FSR_1_024;
    else if (pga_range_found == 2.048) return PGA_FSR_2_048;
    else if (pga_range_found == 4.096) return PGA_FSR_4_096;
    else if (pga_range_found == 6.144) return PGA_FSR_6_144;
    else return PGA_FSR_2_048; // This is a default value for ADS111x.
}

/*
 * Function that configurates ADS
 * to work in a single-shot mode.
 */
esp_err_t ads_config(const uint8_t channel, const uint8_t relative_to, const double max_voltage_input) {
    // first, perform an error-check.
    esp_err_t ret = config_error_check(channel, relative_to);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "config_error_check failed, wrong value provided for (%d)", ret);
        return ret;
    }

    // bits 14:12 and 11:9 of the Config register
    const uint8_t mux_setup = mux_value(channel, relative_to);
    const uint8_t pga_setup = pga_value(max_voltage_input);

    const uint16_t conf_word =  CONFIG_OS_READ << 15 |
                                ((uint16_t)mux_setup) << 12 | // since they are moved by 12 and 9, we need to cast
                                ((uint16_t)pga_setup) << 9 | // those two as uint16_t, otherwise we'd get two zeros.
                                MODE_SINGLE_SHOT << 8 |
                                DR_128_SPS << 5 |
                                COMP_TRADITIONAL << 4 |
                                ACTIVE_LOW << 3 |
                                NON_LATCHING << 2 |
                                DISABLE_COMPARATOR;

    // splitting the configuration into 2 parts
    const uint8_t conf_word_msb = conf_word >> 8; // from most significant bit
    const uint8_t conf_word_lsb = (conf_word & 0xFF); // to the least significant one

    // starting I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "i2c_cmd_link_create failed");
        return ESP_FAIL;
    }
    // sending a Start bit
    i2c_master_start(cmd);

    // sending the ADS111x address and a WRITE bit
    ret = i2c_master_write_byte(cmd, (SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    if (ret != ESP_OK) goto err_finish; // error handling

    // write to the Config Register
    ret = i2c_master_write_byte(cmd, CONFIG_REGISTER, I2C_MASTER_ACK);
    if (ret != ESP_OK) goto err_finish;

    // write the config_word in 2 parts
    // starting from the most significant bit
    ret = i2c_master_write_byte(cmd, conf_word_msb, I2C_MASTER_ACK);
    if (ret != ESP_OK) goto err_finish;

    // ending with the least significant one
    ret = i2c_master_write_byte(cmd, conf_word_lsb, I2C_MASTER_ACK);
    if (ret != ESP_OK) goto err_finish;

    // send a Stop bit
    i2c_master_stop(cmd);

    // begin the command
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_begin failed %d", ret);
        i2c_cmd_link_delete(cmd);
        return ret;
    }

    // delete the link
    i2c_cmd_link_delete(cmd);
    return ret;

// in case of an error during the I2C communication
err_finish:
    i2c_cmd_link_delete(cmd); // delete the I2C link
    ESP_LOGE(TAG, "I2C Error writing byte to a slave %d", ret);
    return ret;
}

/*
 * Once we've configured the Config Register,
 * the OS bit will tell us about its status.
 * After we've set the OS bit as 1 (0b1), it takes
 * 25µs to turn on, then resets itself to 0 (0b0)
 * and starts a single conversion.
 * Afterward, the OS bit will be set back to 1 (0b1)
 * to indicate that the conversion is complete and the
 * conversion result is ready in the Conversion Register
 */
bool os_ready(void) {
    // create an I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "i2c_cmd_link_create failed");
        return false;
    }
    // Start bit
    i2c_master_start(cmd);

    // writing to the slave's address with a write bit
    esp_err_t e = i2c_master_write_byte(cmd, (SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    if (e != ESP_OK) goto err_finish;

    // writing to the config register
    e = i2c_master_write_byte(cmd, CONFIG_REGISTER, I2C_MASTER_ACK);
    if (e != ESP_OK) goto err_finish;

    // Start bit again, because we will be READING a byte
    i2c_master_start(cmd);
    // slave's address as a part of the message frame again, this time with a read bit
    e = i2c_master_write_byte(cmd, (SLAVE_ADDRESS << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
    if (e != ESP_OK) goto err_finish;

    // reading the first byte
    uint8_t msb8_value = 0;
    e = i2c_master_read_byte(cmd, &msb8_value, I2C_MASTER_ACK);
    if (e != ESP_OK) goto err_finish;

    // reading the 2nd byte
    uint8_t lsb8_value = 0;
    e = i2c_master_read_byte(cmd, &lsb8_value, I2C_MASTER_ACK);
    if (e != ESP_OK) goto err_finish;

    // Stop bit
    i2c_master_stop(cmd);
    // begin the command
    e = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    if (e != ESP_OK) goto err_finish;

    // the value of the OS bit
    const uint8_t os_value = msb8_value >> 7;

    // if 1 - the conversion is complete
    if (os_value == 1) {
        return true;
    } // else - conversion still ongoing
    return false;

err_finish:
    ESP_LOGE(TAG, "i2c master operation failed %d", e);
    i2c_cmd_link_delete(cmd);
    return false;
}

/*
 * function for a user, to read the analog value from a
 * pin (channel) of choice. The user provides the
 * max_voltage_input argument which makes the driver choose
 * a proper PGA FSR setting. The 4th argument is a pointer
 * to a uint16_t value, where the read value will be stored
 */
esp_err_t ads_read(const uint8_t channel, const uint8_t relative_to, const double max_voltage_input, int16_t *value) {
    // first, configuration of an ADS111x
    esp_err_t ret = ads_config(channel, relative_to, max_voltage_input);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ads_read failed");
        return ret;
    }

    // wait for the conversion to end
    for (;;) {
        if (os_ready()) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // then perform the reading
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "i2c_cmd_link_create failed");
        return ESP_FAIL;
    }
    // Start bit
    i2c_master_start(cmd);

    // communicate with a slave device, write bit
    ret = i2c_master_write_byte(cmd, (SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    if (ret != ESP_OK) goto err_finish;

    // write to the Conversion Register
    ret = i2c_master_write_byte(cmd, CONVERSION_REGISTER, I2C_MASTER_ACK);
    if (ret != ESP_OK) goto err_finish;

    // Start bit again, we will be reading
    i2c_master_start(cmd);
    // communicate with a slave device (ADS111x)
    ret = i2c_master_write_byte(cmd, (SLAVE_ADDRESS << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
    if (ret != ESP_OK) goto err_finish;

    // variable to store read data from msb
    uint8_t msb_data = 0;
    // read the 1st byte of data
    ret = i2c_master_read_byte(cmd, &msb_data, I2C_MASTER_ACK);
    if (ret != ESP_OK) goto err_finish;

    // variable to store read data to lsb
    uint8_t lsb_data = 0;
    // read the 2nd byte of data
    ret = i2c_master_read_byte(cmd, &lsb_data, I2C_MASTER_NACK);
    if (ret != ESP_OK) goto err_finish;

    // Stop bit
    i2c_master_stop(cmd);
    // begin the command
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) goto err_finish;

    // delete the command link
    i2c_cmd_link_delete(cmd);
    // getting answer from the Conversion Register
    const uint16_t raw = (((uint16_t)msb_data) << 8) | lsb_data;
    // interpreting it as 2's complement
    *value = (int16_t)raw;
    return ret;

err_finish:
    ESP_LOGE(TAG, "i2c master action failed %d", ret);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*
 * The function below is used to convert
 * the digital value to the analog one.
 * Although ADS111x is a 16bit adc, the user
 * will not always be using its full range.
 * If the PGA FSR is set to 4.096V, but the
 * supply voltage is 3.3V, only the voltage
 * up to 3.3-3.6V can be reliably read.
 */
double digital_to_analog(const double vdd, const int16_t value) {
    /* First, we must know our PGA full-scale range.
     * Based on the setting of the ADS's PGA,
     * the size of the LSB changes.
     */
    const uint8_t pga_range = pga_value(vdd);
    double lsb_size = 0;
    // choose the LSB's size according to the docs.
    switch (pga_range) {
        case PGA_FSR_0_256:
            lsb_size = 7.8125e-6;
            break;
        case PGA_FSR_0_512:
            lsb_size = 15.625e-6;
            break;
        case PGA_FSR_1_024:
            lsb_size = 31.25e-6;
            break;
        case PGA_FSR_2_048:
            lsb_size = 62.5e-6;
            break;
        case PGA_FSR_4_096:
            lsb_size = 125e-6;
            break;
        case PGA_FSR_6_144:
            lsb_size = 187.5e-6;
            break;
        default:
            lsb_size = 62.5e-6;
            break;
    }

    // return the analog value
    return value * lsb_size;
}
