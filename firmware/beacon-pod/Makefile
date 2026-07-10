# 031-beacon-camera — ATtiny412 beacon-pod firmware Makefile
#
# Build:   make
# Flash:   make flash CODE_ID=0           (pod A)
#          make flash CODE_ID=1           (pod B)
# Decode:  make decode CSV=path/to/scope.csv  (run scope-trace-decode.py)
# Clean:   make clean
#
# Toolchain (per research.md R2):
#   - avr-gcc + avr-binutils (Debian/Ubuntu: sudo apt install gcc-avr avr-libc binutils-avr)
#   - pymcuprog (pip install pymcuprog)
#   - USB-UART adapter with 4.7 kΩ pull-up on TX line (serialUPDI cable)
#
# Hardware-side:
#   - Microchip ATtiny412 (SOIC-8 or Curiosity Nano dev module for first bring-up)
#   - serialUPDI connection on UPDI pin (see config.h for pin mapping)

MCU     := attiny412
F_CPU   := 20000000UL
CODE_ID ?= 0

CC      := avr-gcc
OBJCOPY := avr-objcopy
SIZE    := avr-size

CFLAGS  := -Wall -Wextra -Wpedantic -std=c11 -Os -mmcu=$(MCU) \
           -DF_CPU=$(F_CPU) -DCODE_ID=$(CODE_ID) \
           -ffunction-sections -fdata-sections \
           -Iinclude
LDFLAGS := -mmcu=$(MCU) -Wl,--gc-sections -Wl,-Map=beacon-pod.map

SRCS := src/main.c src/gold_codes.c
OBJS := $(SRCS:.c=.o)

TARGET := beacon-pod

all: $(TARGET).hex

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^
	$(SIZE) -C --mcu=$(MCU) $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

flash: $(TARGET).hex
	pymcuprog write -t uart -d $(MCU) -u /dev/ttyUSB0 --erase --verify -f $(TARGET).hex

# Scope-trace verification (FR-1.5(a) — runs the Python decoder against a captured CSV)
decode:
	@if [ -z "$(CSV)" ]; then echo "Usage: make decode CSV=path/to/scope.csv"; exit 1; fi
	python3 tests/scope_trace_decode.py --csv $(CSV) --code-id $(CODE_ID)

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).hex $(TARGET).map

.PHONY: all flash decode clean
