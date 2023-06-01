#
# Copyright 2022, UNSW
#
# SPDX-License-Identifier: BSD-2-Clause
#

ifeq ($(strip $(BUILD_DIR)),)
$(error BUILD_DIR must be specified)
endif

ifeq ($(strip $(SEL4CP_SDK)),)
$(error SEL4CP_SDK must be specified)
endif

ifeq ($(strip $(SEL4CP_BOARD)),)
$(error SEL4CP_BOARD must be specified)
endif

ifeq ($(strip $(CAKE_COMPILER)),)
$(error CAKE_COMPILER must be specified)
endif

ifeq ($(strip $(SEL4CP_CONFIG)),)
$(error SEL4CP_CONFIG must be specified)
endif

TOOLCHAIN := aarch64-none-elf

CPU := cortex-a53

CC := $(TOOLCHAIN)-gcc
LD := $(TOOLCHAIN)-ld
AS := $(TOOLCHAIN)-as
SEL4CP_TOOL ?= $(SEL4CP_SDK)/bin/sel4cp

RINGBUFFERDIR=libsharedringbuffer

BOARD_DIR := $(SEL4CP_SDK)/board/$(SEL4CP_BOARD)/$(SEL4CP_CONFIG)

IMAGES := serial.elf serial_server.elf serial_server2.elf mux_tx.elf mux_rx.elf
CFLAGS := -mcpu=$(CPU) -mstrict-align -ffreestanding -g3 -O3 -Wall  -Wno-unused-function
LDFLAGS := -L$(BOARD_DIR)/lib -Llib
LIBS := -lsel4cp -Tsel4cp.ld -lc

IMAGE_FILE = $(BUILD_DIR)/loader.img
REPORT_FILE = $(BUILD_DIR)/report.txt

CFLAGS += -I$(BOARD_DIR)/include \
	-Iinclude	\
	-I$(RINGBUFFERDIR)/include \

SERIAL_OBJS := serial.o libsharedringbuffer/shared_ringbuffer.o
SERIAL_SERVER_OBJS := serial_server.o libsharedringbuffer/shared_ringbuffer.o
SERIAL_SERVER_OBJS2 := serial_server_2.o libsharedringbuffer/shared_ringbuffer.o
MUX_TX_OBJS := mux_tx.o mux_tx_ffi.o libsharedringbuffer/shared_ringbuffer.o
MUX_RX_OBJS := mux_rx.o mux_rx_ffi.o libsharedringbuffer/shared_ringbuffer.o
all: directories $(IMAGE_FILE)

$(BUILD_DIR)/mux_tx.o: $(BUILD_DIR)/mux_tx.S Makefile
	$(CC) -c $(CFLAGS) -mcpu=$(CPU) $< -o $@

$(BUILD_DIR)/mux_tx.S: mux_tx.pk Makefile
	$(CAKE_COMPILER) --target=arm8 --pancake < $< > $@

$(BUILD_DIR)/mux_rx.o: $(BUILD_DIR)/mux_rx.S Makefile
	$(CC) -c $(CFLAGS) -mcpu=$(CPU) $< -o $@

$(BUILD_DIR)/mux_rx.S: mux_rx.pk Makefile
	$(CAKE_COMPILER) --target=arm8 --pancake < $< > $@

$(BUILD_DIR)/%.o: %.c Makefile
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile
	$(AS) -g3 -mcpu=$(CPU) $< -o $@

$(BUILD_DIR)/mux_tx.elf: $(addprefix $(BUILD_DIR)/, $(MUX_TX_OBJS))
	$(LD) $(LDFLAGS) $^ $(LIBS) -o $@

$(BUILD_DIR)/mux_rx.elf: $(addprefix $(BUILD_DIR)/, $(MUX_RX_OBJS))
	$(LD) $(LDFLAGS) $^ $(LIBS) -o $@

$(BUILD_DIR)/serial.elf: $(addprefix $(BUILD_DIR)/, $(SERIAL_OBJS))
	$(LD) $(LDFLAGS) $^ $(LIBS) -o $@

$(BUILD_DIR)/serial_server.elf: $(addprefix $(BUILD_DIR)/, $(SERIAL_SERVER_OBJS))
	$(LD) $(LDFLAGS) $^ $(LIBS) -o $@

$(BUILD_DIR)/serial_server2.elf: $(addprefix $(BUILD_DIR)/, $(SERIAL_SERVER_OBJS2))
	$(LD) $(LDFLAGS) $^ $(LIBS) -o $@

$(IMAGE_FILE) $(REPORT_FILE): $(addprefix $(BUILD_DIR)/, $(IMAGES)) serial.system
	$(SEL4CP_TOOL) serial.system --search-path $(BUILD_DIR) --board $(SEL4CP_BOARD) --config $(SEL4CP_CONFIG) -o $(IMAGE_FILE) -r $(REPORT_FILE)

.PHONY: all depend compile clean

%.o:
	$(CC) $(CFLAGS) -c $(@:.o=.c) -o $@

#Make the Directories
directories:
	$(info $(shell mkdir -p $(BUILD_DIR)/libsharedringbuffer))	\

clean:
	rm -f *.o *.elf .depend*
	find . -name \*.o |xargs --no-run-if-empty rm


	