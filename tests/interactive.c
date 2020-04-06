// Interactive testing for the Nanolab firmware, powered by simavr.

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <simavr/sim_avr.h>
#include <simavr/avr_uart.h>
#include <simavr/sim_elf.h>
#include <simavr/avr_ioport.h>
#include <simavr/avr_spi.h>

#include "sd.h"

#define FREQ 16000000

sd_t sd;
avr_t *avr;

typedef struct arduino_pin {
	bool active_high;
	char *name;
} arduino_pin;

static void pin_change_hook(avr_irq_t *irq, uint32_t value, void *param) {
	arduino_pin *a = (arduino_pin*)param;
	char *desc = value == a->active_high ? "ON" : "OFF";
	printf("%s turned %s\n", a->name, desc);
}

static void sigint_handler(int sig) {
	printf("> ");
	char buffer[1024];
	scanf("%s", buffer);
	if (strlen(buffer) > 64) {
		puts("String longer than serial buffer not supported yet.");
		return;
	}
	if (strcmp(buffer, "quit") == 0) {
		fputs("Saving SD card to disk...", stderr);
		sd_free(&sd);
		fputs("Exiting cleanly!", stderr);
		return exit(0);
	}

	double voltage;
	if (sscanf(buffer, "voltage=%lf", &voltage)) {
		// TODO: set voltage
		printf("TODO: Set voltage to %f\n", voltage);
		return;
	}

	double current;
	if (sscanf(buffer, "current=%lf", &current)) {
		printf("TODO: Set current to %f\n", current);
	}

	int temperature;
	if (sscanf(buffer, "temperature=%d", &temperature)) {
		printf("TODO: Set temperature to %d\n", temperature);
	}

	for (int i = 0; buffer[i] != '\0'; i++) {
		avr_raise_irq(avr_io_getirq(avr,
					    AVR_IOCTL_UART_GETIRQ('0'),
					    UART_IRQ_INPUT),
			      buffer[i]);
	}
}

const struct sigaction sigint_action = {
	.sa_handler = &sigint_handler,
	.sa_mask = 0,
	.sa_flags = 0,
};

int main(int argc, char **argv) {
	/////// "PARSE" COMMAND LINE
	if (argc < 3) {
		fputs("Usage: ./interactive blue_origin.elf sd_card.img", stderr);
		return 1;
	}

	/////// LOAD FIRMWARE
	elf_firmware_t firmware;
	if (elf_read_firmware(argv[1], &firmware)) {
		fputs("Error reading firmware elf.", stderr);
		return 1;
	}

	avr = avr_make_mcu_by_name("atmega328p");
	avr->frequency = FREQ;
	avr_init(avr);
	avr_load_firmware(avr, &firmware);

	/////// INITIALIZE SD CARD
	if (sd_init(&sd, avr, argv[2])) {
		fprintf(stderr, "Error initializing SD card: %s\n",
			strerror(errno));
		return 1;
	}

	avr_connect_irq(
		avr_io_getirq(avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_OUTPUT),
		sd.irq + SD_IRQ_MOSI);
	avr_connect_irq(
		sd.irq + SD_IRQ_MISO,
		avr_io_getirq(avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_INPUT));
	avr_connect_irq(
		// "digital pin 10" is actually PORTB pin 2
		avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('C'), 0),
		sd.irq + SD_IRQ_CS);

	/////// PIN IRQS
#define WATCH_PIN(port, pin, _name, _active_high)			\
	arduino_pin _name = {						\
		.name = #_name,						\
		.active_high = _active_high,				\
	};								\
	avr_irq_register_notify(					\
		avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(port), pin),	\
		pin_change_hook, &_name)

	WATCH_PIN('D', 2, PUMP_POWER, false);
	WATCH_PIN('D', 5, PUMP_1, true);
	WATCH_PIN('D', 6, PUMP_2, true);
	WATCH_PIN('B', 0, SOL_1, true);
	WATCH_PIN('B', 1, SOL_2, true);
	WATCH_PIN('B', 2, SOL_3, true);
	WATCH_PIN('D', 3, MOTOR, true);
	WATCH_PIN('C', 5, EXPERIMENT, false);

	/////// CATCH SIGNALS
	sigaction(SIGINT, &sigint_action, NULL);
	sigaction(SIGTERM, &sigint_action, NULL);

	/////// RUN SIM
	unsigned long cycles_run = 0;
	puts("About to start");
	while (1) {
		int state = avr_run(avr);
		if (state == cpu_Done) {
			fputs("CPU stopped gracefully.", stderr);
			return 0;
		}
		if (state == cpu_Crashed) {
			fputs("CPU crashed.", stderr);
			return 1;
		}
		if (cycles_run++ % FREQ == 0) {
			fprintf(stderr, "seconds: %lu\n", cycles_run / FREQ);
		}
	}
}
