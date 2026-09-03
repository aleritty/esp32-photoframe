#ifndef BOARD_PICPAK_H
#define BOARD_PICPAK_H

#include "driver/gpio.h"

// PicPak Tesserae — ESP32-C3, 4.2" 400x300 4-colour BWRY panel, no PSRAM,
// no SD card, no external RTC / sensor. Pin map from
// varanu5/picpak-tesserae-client firmware/main/board.h.

// Board Info
#define BOARD_HAL_NAME "picpak"
#define BOARD_HAL_TYPE BOARD_TYPE_PICPAK

// Display colour model reported to the webapp (4-colour black/white/red/yellow).
#define BOARD_HAL_DISPLAY_TYPE "bwry"

// Single button on GPIO2 (active-low), shared with the battery-sense ADC
// (ADC1 channel 2). No rotate / clear buttons.
#define BOARD_HAL_WAKEUP_KEY GPIO_NUM_2
#define BOARD_HAL_WAKEUP_KEY_NAME "Button"
#define BOARD_HAL_ROTATE_KEY GPIO_NUM_NC
#define BOARD_HAL_CLEAR_KEY GPIO_NUM_NC

// E-paper SPI bus (no SD card shares it). MISO is wired for panel-ID readback.
#define BOARD_HAL_SPI_SCLK_PIN GPIO_NUM_6
#define BOARD_HAL_SPI_MOSI_PIN GPIO_NUM_3
#define BOARD_HAL_SPI_MISO_PIN GPIO_NUM_4

// E-paper control pins (UC81xx-class panel: separate DC line, single CS).
#define BOARD_HAL_EPD_CS_PIN GPIO_NUM_9
#define BOARD_HAL_EPD_DC_PIN GPIO_NUM_8
#define BOARD_HAL_EPD_RST_PIN GPIO_NUM_10
#define BOARD_HAL_EPD_BUSY_PIN GPIO_NUM_20
#define BOARD_HAL_EPD_CS1_PIN (-1)  // single panel, no dual-CS
#define BOARD_HAL_EPD_ENABLE_PIN (-1)  // no panel power switch

// Status LED (active-high).
#define BOARD_HAL_LED_PIN GPIO_NUM_21
#define BOARD_HAL_LED_INVERTED false

// Panel is native landscape 400x300.
#define BOARD_HAL_DISPLAY_ROTATION_DEG 0

#endif  // BOARD_PICPAK_H
