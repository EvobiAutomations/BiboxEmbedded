TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_PCA10001

# application source
C_SOURCE_FILES += main.c
C_SOURCE_FILES += ble_srv_common.c
C_SOURCE_FILES += ble_nus.c
C_SOURCE_FILES += ble_advdata.c
C_SOURCE_FILES += ble_conn_params.c
C_SOURCE_FILES += softdevice_handler.c
C_SOURCE_FILES += app_timer.c
C_SOURCE_FILES += app_gpiote.c
C_SOURCE_FILES += ble_debug_assert_handler.c
C_SOURCE_FILES += nrf_delay.c
C_SOURCE_FILES += pstorage.c
C_SOURCE_FILES += nrf_pwm.c
C_SOURCE_FILES += ST7735.c
C_SOURCE_FILES += winbond_flash.c
C_SOURCE_FILES += simple_uart.c
C_SOURCE_FILES += MFRC522.c
C_SOURCE_FILES += soft_spi.c
C_SOURCE_FILES += SEGGER_RTT.c
C_SOURCE_FILES += SEGGER_RTT_printf.c
C_SOURCE_FILES += max7219.c
C_SOURCE_FILES += mma8452.c
C_SOURCE_FILES += mp3module.c
C_SOURCE_FILES += twi.c
C_SOURCE_FILES += timeslot.c


SDK_PATH = F:/mukesh/TASKS/sdk6withdisplay/nrf51822/


OUTPUT_FILENAME := bibox

DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

USE_SOFTDEVICE := S110
#USE_SOFTDEVICE := S210

CFLAGS := -DDEBUG_NRF_USER -DBLE_STACK_SUPPORT_REQD -DS110

# we do not use heap in this app
ASMFLAGS := -D__HEAP_SIZE=100

# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections

# let linker to dump unused sections
LDFLAGS := -Wl,--gc-sections
LDFLAGS += --specs=nano.specs




INCLUDEPATHS += -I"../"
INCLUDEPATHS += -I"$(SDK_PATH)Include/s110"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/device_manager"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/ble_services"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"
INCLUDEPATHS += -I"$(SDK_PATH)Board/nrf6310/s110/bibox_tern_flash/app_uart_library_example_with_ble/arm"
INCLUDEPATHS += -I"C:/Keil_v5/ARM/Pack/NordicSemiconductor/RTT"




C_SOURCE_PATHS += $(SDK_PATH)Source/ble
C_SOURCE_PATHS += $(SDK_PATH)Source/ble/device_manager
C_SOURCE_PATHS += $(SDK_PATH)Source/app_common
C_SOURCE_PATHS += $(SDK_PATH)Source/sd_common
C_SOURCE_PATHS += $(SDK_PATH)Board/nrf6310/s110/bibox_tern_flash/app_uart_library_example_with_ble/arm
C_SOURCE_PATHS += C:/Keil_v5/ARM/Pack/NordicSemiconductor/RTT





include $(SDK_PATH)Source/templates/gcc/Makefile.common
