#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include <stdint.h>

// address of the ADS1115 device
#define SLAVE_ADDRESS (0x48)

#define GND (10)
#define CH0 (0)
#define CH1 (1)
#define CH2 (2)
#define CH3 (3)

// Address Pointer Register
#define CONVERSION_REGISTER (0b00)
#define CONFIG_REGISTER     (0b01)

// Config register 15th byte
// shift << 15 later
#define CONFIG_OS_READ      (0b1)
#define CONFIG_OS_NOTHING   (0b0)

// all possible values for bytes 14:12 of
// the config register
// need to shift << 12 later.
#define MUX_P0_N1           (0b000) // default
#define MUX_P0_N3           (0b001)
#define MUX_P1_N3           (0b010)
#define MUX_P2_N3           (0b011)
#define MUX_P0_NGND         (0b100)
#define MUX_P1_NGND         (0b101)
#define MUX_P2_NGND         (0b110)
#define MUX_P3_NGND         (0b111)

// Config reg 11:9 bytes
// full-scale range
// shift << 9 later
#define PGA_FSR_6_144       (0b000)
#define PGA_FSR_4_096       (0b001)
#define PGA_FSR_2_048       (0b010) // default
#define PGA_FSR_1_024       (0b011)
#define PGA_FSR_0_512       (0b100)
#define PGA_FSR_0_256       (0b101)

// Config reg 8th byte
#define MODE_CONTINUOUS     (0b0)
#define MODE_SINGLE_SHOT    (0b1) // default

// Config reg 7:5 bytes
// data rate (samples per second)
#define DR_8_SPS            (0b000)
#define DR_16_SPS           (0b001)
#define DR_32_SPS           (0b010)
#define DR_64_SPS           (0b011)
#define DR_128_SPS          (0b100) // default
#define DR_250_SPS          (0b101)
#define DR_475_SPS          (0b110)
#define DR_860_SPS          (0b111)

// Config reg 4th byte
// Comparator's mode
#define COMP_TRADITIONAL    (0b0) // default
#define COMP_WINDOW         (0b1)

// Config reg 3rd byte
// Comparator polarity
#define ACTIVE_LOW          (0b0) // default
#define ACTIVE_HIGH         (0b1)

// Config reg 2nd byte
// Latching comparator
#define NON_LATCHING         (0b0) // default
#define LATCHING             (0b1)

// Config reg 1:0 bytes
// Comparator queue and disable
#define ASSERT_AFTER_1       (0b00)
#define ASSERT_AFTER_2       (0b01)
#define ASSERT_AFTER_4       (0b10)
#define DISABLE_COMPARATOR   (0b11) // default

esp_err_t ads_init(void);
esp_err_t ads_read(const uint8_t channel, const uint8_t relative_to, uint16_t *value);
