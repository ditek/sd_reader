
NAME := sd-reader
HEX := $(NAME).hex
OUT := $(NAME).out
MAP := $(NAME).map
SOURCES := $(wildcard *.c)
HEADERS := $(wildcard *.h)
OBJECTS := $(patsubst %.c,%.o,$(SOURCES))

MCU := atmega32u4
MCU_AVRDUDE := m168
MCU_FREQ := 8000000UL
BOARD := TEENSY2

CC := avr-gcc
OBJCOPY := avr-objcopy
SIZE = avr-size -A --format=avr --mcu=$(MCU)
DOXYGEN := doxygen

CFLAGS := -Wall -pedantic -mmcu=$(MCU) -std=c99 -g -Os -DF_CPU=$(MCU_FREQ) -DBOARD=$(BOARD)

all: $(HEX)

clean:
	rm -f $(HEX) $(OUT) $(MAP) $(OBJECTS)
	rm -rf doc/html

flash: $(HEX)
	#avrdude -y -c avr910 -p $(MCU_AVRDUDE) -U flash:w:$(HEX)
	avrdude -p $(MCU_AVRDUDE) -P usb -c avrispmkII -V -U flash:w:$(HEX)

$(HEX): $(OUT)
	$(OBJCOPY) -R .eeprom -O ihex $< $@

$(OUT): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ -Wl,-Map,$(MAP) $^
	@echo
	@$(SIZE) $@
	@echo

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

%.pp: %.c
	$(CC) $(CFLAGS) -E -o $@ $<

%.ppo: %.c
	$(CC) $(CFLAGS) -E $<

doc: $(HEADERS) $(SOURCES) Doxyfile
	$(DOXYGEN) Doxyfile

.PHONY: all clean flash doc

