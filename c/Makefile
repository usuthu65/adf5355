# Build the Linux-userspace example against a checkout of analogdevicesinc/no-OS.

NO_OS_DIR ?= ../../no-OS
CC ?= gcc

CPPFLAGS += -D_DEFAULT_SOURCE \
            -I$(NO_OS_DIR)/include \
            -I$(NO_OS_DIR)/drivers/frequency/adf5355 \
            -I$(NO_OS_DIR)/drivers/platform/linux-userspace
CFLAGS ?= -O2 -Wall -Wextra -std=c11

TARGET := adf5355_noos_example
SOURCES := adf5355_noos_example.c \
           $(NO_OS_DIR)/drivers/frequency/adf5355/adf5355.c \
           $(NO_OS_DIR)/drivers/platform/linux-userspace/linux_spi.c \
           $(NO_OS_DIR)/drivers/platform/linux-userspace/linux_delay.c \
           $(NO_OS_DIR)/util/no_os_alloc.c \
           $(NO_OS_DIR)/util/no_os_util.c

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(SOURCES)

clean:
	rm -f $(TARGET)
