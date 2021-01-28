LIB_NAME=libstm32f0xx.a

SRC_DIR=\
CMSIS/CM0/CoreSupport/ \
STM32F0xx_StdPeriph_Driver/src/

OBJ_DIR=obj/

CROSS_COMPILE?=arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar

CHIP=STM32F0XX_LD

CONFIG_FILE=stm32f0xx_conf.h

INCLUDES=-include$(CONFIG_FILE)
INCLUDES+=-I.
INCLUDES+=-ISTM32F0xx_StdPeriph_Driver/inc/
INCLUDES+=-ICMSIS/CM0/DeviceSupport/ST/STM32F0xx/
INCLUDES+=-ICMSIS/CM0/CoreSupport/

CFLAGS=-g -Wall -Werror -O3
CFLAGS+=-D$(CHIP) $(INCLUDES) -MMD -MP
CFLAGS+=-mcpu=cortex-m0 -mthumb
CFLAGS+=-ffreestanding -nostdlib

vpath %.c $(SRC_DIR)

SRCS_PERIPH=\
stm32f0xx_adc.c   stm32f0xx_dac.c     stm32f0xx_i2c.c   stm32f0xx_spi.c \
stm32f0xx_can.c   stm32f0xx_dbgmcu.c  stm32f0xx_iwdg.c  stm32f0xx_syscfg.c \
stm32f0xx_cec.c   stm32f0xx_dma.c     stm32f0xx_misc.c  stm32f0xx_tim.c \
stm32f0xx_comp.c  stm32f0xx_exti.c    stm32f0xx_pwr.c   stm32f0xx_usart.c \
stm32f0xx_crc.c   stm32f0xx_flash.c   stm32f0xx_rcc.c   stm32f0xx_wwdg.c \
stm32f0xx_crs.c   stm32f0xx_gpio.c    stm32f0xx_rtc.c

SRCS=$(SRCS_PERIPH)

OBJS=$(addprefix $(OBJ_DIR),$(SRCS:.c=.o))
DEPS=$(OBJS:%.o=%.d)

all: dirs $(LIB_NAME)

dirs:
	mkdir -p $(OBJ_DIR)

$(LIB_NAME): $(OBJS)
	$(AR) -r $@ $^

$(OBJ_DIR)%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

-include $(DEPS)

clean:
	rm -rf $(LIB_NAME) $(OBJ_DIR)
