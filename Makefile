sketch      := blue_origin_fc
sketch_elf  := blue_origin_fc/blue_origin_fc.arduino.avr.uno.elf
interactive := tests/interactive
int_libs    := $(patsubst %.c,%.o,$(wildcard tests/lib/*.c))

CFLAGS      := $(CFLAGS) -Itests/lib -g -Wall

all: $(interactive) $(sketch_elf)

$(sketch_elf): $(sketch)/$(sketch).ino
	arduino-cli compile -b 'arduino:avr:uno' $(sketch)

$(interactive): $(interactive).o $(int_libs)
	$(CC) $(CFLAGS) -o $@ -lsimavr -lelf -lutil -pthread \
		$(interactive).o $(int_libs)

clean:
	rm -f $(sketch_elf) $(interactive) tests/lib/*.o tests/*.o

.PHONY: clean all
