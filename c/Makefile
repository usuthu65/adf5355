# Build the Linux-userspace example against a checkout of analogdevicesinc/no-OS.

NO_OS_DIR ?= ../../no-OS
CC ?= gcc

CPPFLAGS += -D_DEFAULT_SOURCE \
            -I$(NO_OS_DIR)/include \
            -I$(NO_OS_DIR)/drivers/frequency/adf5355 \
            -I$(NO_OS_DIR)/drivers/platform/linux-userspace
CFLAGS ?= -O2 -Wall -Wextra -Werror -std=c11

# These diagnostics originate in ADI's generic no-OS sources, not in this
# application. They are intentional: API-shaped no-op mutexes, an unused
# argument in the rate-rounding hook, and an informational Linux SPI notice
# about chip-select delays that this application leaves at zero.
NO_OS_CFLAGS := -Wno-unused-parameter -Wno-cpp

TARGET := adf5355_noos_example
BUILD_DIR := build

APP_SOURCE := adf5355_noos_example.c
APP_OBJECT := $(BUILD_DIR)/adf5355_noos_example.o

NO_OS_SOURCES := $(NO_OS_DIR)/drivers/frequency/adf5355/adf5355.c \
                 $(NO_OS_DIR)/drivers/api/no_os_spi.c \
                 $(NO_OS_DIR)/drivers/platform/linux-userspace/linux_spi.c \
                 $(NO_OS_DIR)/drivers/platform/linux-userspace/linux_delay.c \
                 $(NO_OS_DIR)/util/no_os_alloc.c \
                 $(NO_OS_DIR)/util/no_os_mutex.c \
                 $(NO_OS_DIR)/util/no_os_util.c
NO_OS_OBJECTS := $(patsubst $(NO_OS_DIR)/%.c,$(BUILD_DIR)/no-os/%.o,$(NO_OS_SOURCES))

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(APP_OBJECT) $(NO_OS_OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^

$(APP_OBJECT): $(APP_SOURCE)
	mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/no-os/%.o: $(NO_OS_DIR)/%.c
	mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(NO_OS_CFLAGS) -c -o $@ $<

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

