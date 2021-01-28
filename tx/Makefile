# Copyright (C) 2021 WMIC authors
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3.

APP_NAME=tx_fw

SRC_DIR=./
OBJ_DIR=obj/
LIB_DIR=../libs/
INCLUDE_DIR=../
CROSS_COMPILE?=arm-none-eabi-
APP=$(OBJ_DIR)$(APP_NAME)

SPL_PATH=$(LIB_DIR)spl/
SPL_DEVICE_SUPPORT_PATH=$(SPL_PATH)CMSIS/CM0/DeviceSupport/ST/STM32F0xx/
SPL_CORE_SUPPORT=$(SPL_PATH)CMSIS/CM0/CoreSupport/
SPL_PERIPH_PATH=$(SPL_PATH)STM32F0xx_StdPeriph_Driver/
SPL_LIB=stm32f0xx
SPL_CONFIG_FILE=$(SPL_PATH)stm32f0xx_conf.h
SPL_FLAGS=-DSTM32F0XX_LD

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump
SIZE=$(CROSS_COMPILE)size

INCLUDES=-include$(SPL_CONFIG_FILE)
INCLUDES+=-I$(INCLUDE_DIR)
INCLUDES+=-I$(SPL_CORE_SUPPORT)
INCLUDES+=-I$(SPL_DEVICE_SUPPORT_PATH)
INCLUDES+=-I$(SPL_PATH)
INCLUDES+=-I$(SPL_PERIPH_PATH)inc
INCLUDES+=-I$(SRC_DIR)

CFLAGS=-g -Wall -Werror -O0
CFLAGS+=$(INCLUDES) -MMD -MP
CFLAGS+=-ffunction-sections -fdata-sections
CFLAGS+=-mcpu=cortex-m0 -mthumb
CFLAGS+=$(SPL_FLAGS)

LDFLAGS=-mcpu=cortex-m0 -mthumb -Wl,--gc-sections -Wl,-Map=$(APP).map

vpath %.c $(SRC_DIR) $(SPL_DEVICE_SUPPORT_PATH) $(SRC_BSP_DIR)
vpath %.s $(SRC_DIR)

STARTUP=startup_stm32f0xx.s
SRCS=system_stm32f0xx.c stm32_it.c syscalls.c \
     main.c uart.c jtag.c delay.c gpio.c i2c.c \
     led.c tx.c

OBJS=$(addprefix $(OBJ_DIR),$(SRCS:.c=.o)) \
  $(addprefix $(OBJ_DIR),$(STARTUP:.s=.o))
DEPS=$(OBJS:%.o=%.d)

LINKER_SCRIPT=$(SRC_DIR)stm32_flash.ld

all: dirs $(APP).elf

dirs:
	mkdir -p $(OBJ_DIR)

$(APP).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ -L$(SPL_PATH) -l$(SPL_LIB) \
	  -T$(LINKER_SCRIPT)
	$(OBJCOPY) --pad-to 0x08008000 -O ihex $(APP).elf $(APP).hex
	$(OBJCOPY) -O binary $(APP).elf $(APP).bin
	$(OBJDUMP) -St $(APP).elf > $(APP).lst
	$(SIZE) $(APP).elf

$(OBJ_DIR)%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@ 

$(OBJ_DIR)%.o: %.s
	$(CC) -c $(CFLAGS) $< -o $@

-include $(DEPS)

clean:
	rm -rf $(OBJ_DIR)

program:
	st-flash write $(APP).bin 0x08000000
