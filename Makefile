# Copyright (C) 2020-2022 Frand Ren
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3.

TX_NAME=tx_fw
RX_NAME=rx_fw
TX_GEN_NAME=tx
RX_GEN_NAME=rx

TX_DIR=tx/
TX_OBJ_DIR=$(TX_DIR)obj/
RX_DIR=rx/
RX_OBJ_DIR=$(RX_DIR)obj/

LIB_DIR=libs/
SPL_PATH=$(LIB_DIR)spl/

OUTPUT_DIR=output/

CROSS_COMPILE?=arm-none-eabi-
OBJCOPY=$(CROSS_COMPILE)objcopy

.PHONY: tx rx libs

all: tx rx

tx: libs
	$(MAKE) -C $(TX_DIR)
	mkdir -p $(OUTPUT_DIR)

	cp $(TX_OBJ_DIR)$(TX_NAME).hex $(OUTPUT_DIR)$(TX_GEN_NAME).hex
	cp $(TX_OBJ_DIR)$(TX_NAME).elf $(OUTPUT_DIR)$(TX_GEN_NAME).elf

	$(OBJCOPY) --input-target=ihex --output-target=binary \
	  $(OUTPUT_DIR)$(TX_GEN_NAME).hex $(OUTPUT_DIR)$(TX_GEN_NAME).bin

rx: libs
	$(MAKE) -C $(RX_DIR)
	mkdir -p $(OUTPUT_DIR)

	cp $(RX_OBJ_DIR)$(RX_NAME).hex $(OUTPUT_DIR)$(RX_GEN_NAME).hex
	cp $(RX_OBJ_DIR)$(RX_NAME).elf $(OUTPUT_DIR)$(RX_GEN_NAME).elf

	$(OBJCOPY) --input-target=ihex --output-target=binary \
	  $(OUTPUT_DIR)$(RX_GEN_NAME).hex $(OUTPUT_DIR)$(RX_GEN_NAME).bin

libs:
	$(MAKE) -C $(SPL_PATH)

clean:
	$(MAKE) -C $(TX_DIR) clean
	$(MAKE) -C $(RX_DIR) clean
	rm -rf $(OUTPUT_DIR)

distclean:
	$(MAKE) -C $(TX_DIR) clean
	$(MAKE) -C $(RX_DIR) clean
	$(MAKE) -C $(SPL_PATH) clean
	rm -rf $(OUTPUT_DIR)

erase:
	st-flash erase

flashtx:
	st-flash write $(OUTPUT_DIR)$(TX_GEN_NAME).bin 0x08000000

flashrx:
	st-flash write $(OUTPUT_DIR)$(RX_GEN_NAME).bin 0x08000000
