// Board HAL for the PicPak Tesserae (ESP32-C3, 4.2" 400x300 BWRY panel).
//
// No PMIC, no SD card, no external RTC / sensor. Single button on GPIO2 shared
// with the battery-sense ADC (ADC1 channel 2) — reading the battery briefly
// reconfigures the pin to analog, then it is restored to a pulled-up input for
// the button / deep-sleep wake. USB presence is inferred from USB-Serial-JTAG
// (PC connections only — chargers / power banks are not detected, same caveat
// as the Seeed XIAO boards).

#include "battery_adc.h"
#include "board_hal.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/usb_serial_jtag.h"
#include "epaper.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "board_hal_picpak";

// Battery sense: ADC1 channel 2 == GPIO2. voltage_mv = calibrated_pin_mv *
// BATT_DIVIDER. The divider ratio is empirically calibrated (see the PicPak
// docs); a full 4.2 V cell reads ~2897 mV on the pin at 12 dB attenuation.
#define VBAT_ADC_CHANNEL ADC_CHANNEL_2
#define VBAT_DIVIDER 1.45f

// Per-unit correction for divider tolerance, measured with a multimeter:
// (multimeter_mV / firmware_reported_mV). 1.0 = none.
#ifndef VBAT_CAL_SCALE
#define VBAT_CAL_SCALE 1.0f
#endif

// Restore GPIO2 to a pulled-up digital input after an ADC read so it works as
// the active-low button and the deep-sleep wake source.
static void restore_button_pin(void)
{
    gpio_reset_pin(BOARD_HAL_WAKEUP_KEY);
    gpio_set_direction(BOARD_HAL_WAKEUP_KEY, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOARD_HAL_WAKEUP_KEY, GPIO_PULLUP_ONLY);
}

esp_err_t board_hal_init(void)
{
    ESP_LOGI(TAG, "Initializing PicPak Board HAL");

    // Status LED (active-high), off by default.
    gpio_config_t led_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BOARD_HAL_LED_PIN),
    };
    gpio_config(&led_conf);
    gpio_set_level(BOARD_HAL_LED_PIN, 0);

    restore_button_pin();

    // SPI bus for the e-paper panel (nothing else shares it).
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = BOARD_HAL_SPI_MOSI_PIN,
        .miso_io_num = BOARD_HAL_SPI_MISO_PIN,
        .sclk_io_num = BOARD_HAL_SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 400 * 300 / 4 + 100,  // one 2bpp full frame
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    epaper_config_t ep_cfg = {
        .spi_host = SPI2_HOST,
        .pin_cs = BOARD_HAL_EPD_CS_PIN,
        .pin_dc = BOARD_HAL_EPD_DC_PIN,
        .pin_rst = BOARD_HAL_EPD_RST_PIN,
        .pin_busy = BOARD_HAL_EPD_BUSY_PIN,
        .pin_cs1 = BOARD_HAL_EPD_CS1_PIN,
        .pin_enable = BOARD_HAL_EPD_ENABLE_PIN,
    };
    epaper_init(&ep_cfg);

    return ESP_OK;
}

esp_err_t board_hal_prepare_for_sleep(void)
{
    ESP_LOGI(TAG, "Preparing PicPak for sleep");
    epaper_enter_deepsleep();
    gpio_set_level(BOARD_HAL_LED_PIN, 0);
    restore_button_pin();
    return ESP_OK;
}

int board_hal_get_battery_voltage(void)
{
    // Create → read → destroy per call: the ADC and the button share GPIO2, so
    // the pin can't be left in analog mode. Called rarely (once per wake).
    battery_adc_config_t cfg = {
        .unit = ADC_UNIT_1,
        .channel = VBAT_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .enable_pin = -1,
        .settle_ms = 0,
        .samples = 16,
        .divider = VBAT_DIVIDER,
        .cal_scale = VBAT_CAL_SCALE,
    };
    battery_adc_t *adc = NULL;
    if (battery_adc_create(&cfg, &adc) != ESP_OK) {
        restore_button_pin();
        return -1;
    }
    int mv = battery_adc_read_mv(adc);
    battery_adc_destroy(adc);
    restore_button_pin();
    return mv;
}

bool board_hal_is_battery_connected(void)
{
    return board_hal_get_battery_voltage() > 2500;
}

int board_hal_get_battery_percent(void)
{
    int voltage = board_hal_get_battery_voltage();
    if (voltage < 0)
        return -1;
    if (voltage >= 4200)
        return 100;
    if (voltage <= 3300)
        return 0;
    return (voltage - 3300) * 100 / (4200 - 3300);
}

bool board_hal_is_charging(void)
{
    // No charge-status line on this board; USB presence is the best proxy
    // (PC / USB-host only — see file header).
    return usb_serial_jtag_is_connected();
}

bool board_hal_is_usb_connected(void)
{
    return usb_serial_jtag_is_connected();
}

void board_hal_shutdown(void)
{
    ESP_LOGI(TAG, "No hard power-off on PicPak; entering deep sleep");
    board_hal_prepare_for_sleep();
    esp_deep_sleep_start();
}

esp_err_t board_hal_rtc_init(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t board_hal_rtc_get_time(time_t *t)
{
    (void) t;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t board_hal_rtc_set_time(time_t t)
{
    (void) t;
    return ESP_ERR_NOT_SUPPORTED;
}

bool board_hal_rtc_is_available(void)
{
    return false;
}

esp_err_t board_hal_get_temperature(float *t)
{
    (void) t;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t board_hal_get_humidity(float *h)
{
    (void) h;
    return ESP_ERR_NOT_SUPPORTED;
}

void board_hal_led_set(board_hal_led_t led, bool on)
{
    // One physical LED; both logical LEDs map to it.
    (void) led;
    gpio_set_level(BOARD_HAL_LED_PIN, on ? 1 : 0);
}
