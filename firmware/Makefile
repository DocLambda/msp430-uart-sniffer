BINARY  = sniffer
MCU     = msp430g2553

CFLAGS  = -mmcu=$(MCU) -g -Os -Wall
LDFLAGS = -mmcu=$(MCU)

CC      = msp430-gcc
STRIP   = msp430-strip
SIZE    = msp430-size
FLASH   = mspdebug rf2500
RM      = /bin/rm -rf

SOURCES  := $(wildcard *.c)
OBJECTS  := $(SOURCES:%.c=%.o)

.PHONEY: all
all: $(BINARY) size

$(BINARY): $(OBJECTS)
	@echo "LD  $@"
	@$(CC) $(LDFLAGS) -o $@ $(OBJECTS)

.PHONEY: strip
strip: $(BINARY)
	@echo "STRIP $<"
	@$(STRIP) $(BINARY)

.PHONEY: size
size: $(BINARY)
	@echo "Size:"
	@$(SIZE) $(BINARY)

.PHONEY: clean
clean:
	@$(RM) $(wildcard *~)
	@$(RM) $(OBJECTS)
	@$(RM) $(BINARY)
	@echo "Clean."

.PHONEY: flash
flash: $(BINARY)
	$(FLASH) "prog $(BINARY)"

%.o: %.c
	@echo "CC  $<"
	@$(CC) $(CFLAGS) -c -o $@ $<
