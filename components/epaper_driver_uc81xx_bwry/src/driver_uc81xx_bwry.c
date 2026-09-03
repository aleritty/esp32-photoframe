// UC81xx / SSD2683-class 4-colour BWRY e-paper driver.
//
// Implements the epaper.h interface for a 4.2" 400x300 black/white/red/yellow
// panel driven directly over SPI (SCLK/MOSI/CS/DC/RST/BUSY), as on the PicPak
// Tesserae board. The panel consumes a 2-bit-per-pixel framebuffer (4 px/byte,
// MSB = leftmost pixel); this driver repacks the codebase's 4bpp GUI_Paint
// nibble buffer before streaming.
//
// The init sequence + command set are ported from
// varanu5/picpak-tesserae-client (firmware/main/epd_driver.c, epd_init_seq.h),
// AGPL-3.0-or-later. Only the "native MTP" waveform is used here — the vendor
// 5s/10s fast LUTs can be added later if refresh speed matters.
//
// HARDWARE-VALIDATION NOTES (tune on the real panel):
//   - PANEL_CODE[]: the panel's 2-bit code for each ink. Assumed identical to
//     the GUI_Paint nibble codes (0=black,1=white,2=yellow,3=red). If colours
//     come out swapped, remap here.
//   - If the image is mirrored / offset, check TRES (0x61) and the GSST window
//     (0x65) in EPD_INIT_SPECIFIC, and the row order in epaper_display().

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "epaper.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

static const char *TAG = "epaper_uc81xx_bwry";

#define EPD_WIDTH 400
#define EPD_HEIGHT 300
// 2 bits per pixel, 4 pixels per byte.
#define EPD_FB_BYTES (EPD_WIDTH * EPD_HEIGHT / 4)
// SPI clock: the reference driver runs this panel at a conservative 1 MHz.
#define EPD_SPI_HZ (1 * 1000 * 1000)
#define EPD_BUSY_TIMEOUT_MS 40000

// GUI_Paint ink nibble -> panel 2-bit code. Nibbles 0..3 are black/white/
// yellow/red (see epaper.h EPD_7IN3E_*); 4..6 (unused / blue / green) fall
// back to white so a stray dithered pixel can't render as noise.
static const uint8_t PANEL_CODE[16] = {
    0x0,  // 0 black
    0x1,  // 1 white
    0x2,  // 2 yellow
    0x3,  // 3 red
    0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
};

// Native-MTP panel init. Row format { cmd, n_data, data[0..n-1] }; iterate by
// size (0xFF is a valid command byte, not a terminator). Power-ON (0x04) and
// Display-Refresh (0x12) are issued by epaper_display(), not from this table.
// Ported from picpak-tesserae-client epd_init_seq.h (EPD_INIT_SPECIFIC).
static const uint8_t EPD_INIT_SPECIFIC[] = {
    0x00, 2, 0x07, 0x29,              // PSR  — panel setting
    0x01, 2, 0x07, 0x00,             // PWR  — power setting
    0x06, 4, 0x0F, 0x8B, 0x9C, 0x96, // BTST — booster soft-start
    0x30, 1, 0x08,                   // PLL  — clock
    0x50, 1, 0x37,                   // CDI  — VCOM & data interval
    0x61, 4, 0x01, 0x90, 0x01, 0x2C, // TRES — resolution 400x300
    0x65, 4, 0x00, 0x00, 0x00, 0x00, // GSST — window start
    0xE7, 1, 0x96,                   // vendor
    0xE9, 1, 0x01,                   // vendor
    0xFF, 1, 0xA5,                   // vendor
};

static epaper_config_t s_cfg;
static spi_device_handle_t s_spi = NULL;
// DMA-capable internal-RAM repack buffer (2bpp panel framebuffer).
static uint8_t *s_fb = NULL;

#ifdef CONFIG_PM_ENABLE
static esp_pm_lock_handle_t s_pm_lock = NULL;
#endif

static void epd_cmd(uint8_t c)
{
    gpio_set_level(s_cfg.pin_dc, 0);  // DC low = command
    spi_transaction_t t = {.length = 8, .tx_buffer = &c};
    spi_device_polling_transmit(s_spi, &t);
}

static void epd_data(const uint8_t *d, int n)
{
    if (n <= 0) {
        return;
    }
    gpio_set_level(s_cfg.pin_dc, 1);  // DC high = data
    for (int off = 0; off < n; off += 512) {
        int chunk = (n - off > 512) ? 512 : (n - off);
        spi_transaction_t t = {.length = 8 * chunk, .tx_buffer = d + off};
        spi_device_polling_transmit(s_spi, &t);
        // Long full-frame pushes would starve the idle task / trip its WDT.
        if ((off & 8191) == 0) {
            vTaskDelay(1);
        }
    }
}

static void epd_send(uint8_t cmd, const uint8_t *data, int n)
{
    epd_cmd(cmd);
    epd_data(data, n);
}

// UC81xx BUSY is active-low: the panel is busy while the line reads 0.
static void epd_wait_busy(const char *label)
{
    int64_t deadline = esp_timer_get_time() + (int64_t) EPD_BUSY_TIMEOUT_MS * 1000;
    vTaskDelay(pdMS_TO_TICKS(10));
    while (gpio_get_level(s_cfg.pin_busy) == 0) {
        if (esp_timer_get_time() > deadline) {
            ESP_LOGW(TAG, "[%s] BUSY timeout after %d ms", label, EPD_BUSY_TIMEOUT_MS);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void epd_reset(void)
{
    gpio_hold_dis(s_cfg.pin_rst);
    gpio_hold_dis(s_cfg.pin_dc);
    gpio_set_level(s_cfg.pin_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(s_cfg.pin_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(s_cfg.pin_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
}

static void epd_init_native(void)
{
    for (size_t i = 0; i < sizeof(EPD_INIT_SPECIFIC);) {
        uint8_t cmd = EPD_INIT_SPECIFIC[i++];
        uint8_t len = EPD_INIT_SPECIFIC[i++];
        epd_send(cmd, &EPD_INIT_SPECIFIC[i], len);
        i += len;
    }
}

// Repack the 4bpp GUI_Paint nibble buffer (2 px/byte, high nibble = left pixel)
// into the panel's 2bpp format (4 px/byte, bits 7-6 = leftmost pixel).
static void repack_4bpp_to_2bpp(const uint8_t *src)
{
    const int src_row = EPD_WIDTH / 2;   // bytes per row, 4bpp
    const int dst_row = EPD_WIDTH / 4;   // bytes per row, 2bpp
    for (int y = 0; y < EPD_HEIGHT; y++) {
        const uint8_t *s = src + (size_t) y * src_row;
        uint8_t *d = s_fb + (size_t) y * dst_row;
        for (int xb = 0; xb < dst_row; xb++) {
            uint8_t n0 = s[xb * 2] >> 4;
            uint8_t n1 = s[xb * 2] & 0x0F;
            uint8_t n2 = s[xb * 2 + 1] >> 4;
            uint8_t n3 = s[xb * 2 + 1] & 0x0F;
            d[xb] = (uint8_t) ((PANEL_CODE[n0] << 6) | (PANEL_CODE[n1] << 4) |
                               (PANEL_CODE[n2] << 2) | PANEL_CODE[n3]);
        }
    }
}

// --- epaper.h interface ---

void epaper_init(const epaper_config_t *cfg)
{
    s_cfg = *cfg;

    ESP_LOGI(TAG, "Initializing UC81xx BWRY driver (%dx%d, 2bpp)", EPD_WIDTH, EPD_HEIGHT);

    gpio_config_t out_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << s_cfg.pin_dc) | (1ULL << s_cfg.pin_rst),
    };
    gpio_config(&out_conf);
    gpio_config_t in_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << s_cfg.pin_busy),
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&in_conf);
    gpio_set_level(s_cfg.pin_dc, 0);
    gpio_set_level(s_cfg.pin_rst, 1);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = EPD_SPI_HZ,
        .mode = 0,
        .spics_io_num = s_cfg.pin_cs,  // hardware CS (no SD card sharing this bus)
        .queue_size = 4,
    };
    esp_err_t ret = spi_bus_add_device(s_cfg.spi_host, &devcfg, &s_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return;
    }

    s_fb = heap_caps_malloc(EPD_FB_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_fb) {
        ESP_LOGE(TAG, "failed to alloc %d-byte panel framebuffer", EPD_FB_BYTES);
    }

#ifdef CONFIG_PM_ENABLE
    if (esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "epd_update", &s_pm_lock) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PM lock");
    }
#endif
}

void epaper_set_temperature(int8_t celsius)
{
    (void) celsius;  // native MTP waveform: no host temperature compensation
}

void epaper_display(uint8_t *image)
{
    if (!s_spi || !image || !s_fb) {
        ESP_LOGE(TAG, "epaper_display: driver not ready");
        return;
    }

#ifdef CONFIG_PM_ENABLE
    if (s_pm_lock) {
        esp_pm_lock_acquire(s_pm_lock);
    }
#endif

    repack_4bpp_to_2bpp(image);

    epd_reset();
    epd_wait_busy("reset");
    epd_init_native();
    epd_wait_busy("init");

    epd_cmd(0x10);  // DTM: framebuffer load
    epd_data(s_fb, EPD_FB_BYTES);

    epd_cmd(0x04);  // POWER_ON
    epd_wait_busy("power_on");
    epd_cmd(0x12);  // DISPLAY_REFRESH
    epd_wait_busy("refresh");

    uint8_t pof = 0x00;
    epd_send(0x02, &pof, 1);  // POWER_OFF (carries a 0x00 byte on this controller)
    epd_wait_busy("power_off");

    // DEEP_SLEEP needs the 0xA5 check-code or the controller ignores it and
    // keeps drawing between wakes.
    uint8_t dslp = 0xA5;
    epd_send(0x07, &dslp, 1);

#ifdef CONFIG_PM_ENABLE
    if (s_pm_lock) {
        esp_pm_lock_release(s_pm_lock);
    }
#endif

    ESP_LOGI(TAG, "display update complete");
}

void epaper_clear(uint8_t *image, uint8_t color)
{
    if (!image) {
        return;
    }
    uint8_t packed = (uint8_t) ((color << 4) | (color & 0x0F));
    memset(image, packed, (size_t) EPD_WIDTH / 2 * EPD_HEIGHT);
    epaper_display(image);
}

void epaper_enter_deepsleep(void)
{
    if (!s_spi) {
        return;
    }
    // epaper_display() already power-offs + deep-sleeps the panel after every
    // refresh; re-send to be safe (e.g. sleeping without a preceding display).
    uint8_t pof = 0x00;
    epd_send(0x02, &pof, 1);
    epd_wait_busy("deepsleep_power_off");
    uint8_t dslp = 0xA5;
    epd_send(0x07, &dslp, 1);

    // Latch panel-facing pins low so they don't float / back-feed during the
    // MCU's deep sleep.
    gpio_set_level(s_cfg.pin_dc, 0);
    gpio_set_level(s_cfg.pin_rst, 0);
    gpio_hold_en(s_cfg.pin_dc);
    gpio_hold_en(s_cfg.pin_rst);
    gpio_deep_sleep_hold_en();
}

uint16_t epaper_get_width(void)
{
    return EPD_WIDTH;
}

uint16_t epaper_get_height(void)
{
    return EPD_HEIGHT;
}
