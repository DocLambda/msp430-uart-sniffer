BINARY       = msp430-uart-sniffer
FIRMWARE_DIR = firmware

CFLAGS       = -std=gnu99 -Wall
LDFLAGS      = 

MAKE         = /usr/bin/make
INSTALL      = /usr/bin/install
RM           = /bin/rm -rf

SOURCES  := $(wildcard *.c)
OBJECTS  := $(SOURCES:%.c=%.o)

.PHONEY: all
all: $(BINARY) firmware

###############################################################################
# Host part                                                                   #
###############################################################################
$(BINARY): $(OBJECTS)
	@echo "LD     $@"
	@$(CC) $(LDFLAGS) -o $@ $(OBJECTS)

%.o: %.c
	@echo "CC     $<"
	@$(CC) $(CFLAGS) -c -o $@ $<

.PHONEY: clean
clean: clean_firmware
	@$(RM) $(wildcard *~)
	@$(RM) $(OBJECTS)
	@$(RM) $(BINARY)
	@echo "Clean."

.PHONEY: install
install: $(BINARY)
	@echo "INSTALL  $(BINARY)  -->  $(DESTDIR)/usr/bin/$(BINARY)"
	@$(INSTALL) -D $(BINARY) $(DESTDIR)/usr/bin/$(BINARY)

.PHONEY: uninstall
uninstall:
	@echo "UNINSTALL  $(DESTDIR)/usr/bin/$(BINARY)"
	@$(RM) $(DESTDIR)/usr/bin/$(BINARY)
###############################################################################

###############################################################################
# Target firmware part                                                        #
###############################################################################
.PHONY: $(FIRMWARE_DIR) clean_firmware flash_firmware
clean_firmware:
	@$(MAKE) -w -C $(@:clean_%=%) clean

firmware:
	@$(MAKE) -w -C $@

flash_firmware:
	@$(MAKE) -w -C $(@:flash_%=%) flash
###############################################################################

