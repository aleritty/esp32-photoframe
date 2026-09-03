set(srcs "")
if(CONFIG_EP_DRIVER_UC81XX_BWRY)
    list(APPEND srcs "src/driver_uc81xx_bwry.c")
endif()

idf_component_register(
    SRCS "${srcs}"
    REQUIRES driver esp_driver_gpio esp_timer esp_driver_spi epaper
)
