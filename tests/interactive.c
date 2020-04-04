// Interactive testing for the Nanolab firmware, powered by simavr.

#include <stdio.h>
#include <stdlib.h>
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

char buffer[256];
unsigned char buffer_idx = 0;

static void sigint_handler(int sig) {
	printf("> ");
	scanf("%s", buffer);
	buffer_idx = 0;
	if (strcmp(buffer, "quit") == 0) {
		fputs("Saving SD card to disk...", stderr);
		sd_free(&sd);
		fputs("Exiting cleanly!", stderr);
		exit(0);
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

	/////// INITIALIZE UART_PTY
	/* uart_pty_init(avr, &uart_pty); */
	/* uart_pty_connect(&uart_pty, '0'); */

	/////// CATCH SIGNALS
	sigaction(SIGINT, &sigint_action, NULL);
	sigaction(SIGTERM, &sigint_action, NULL);

	/////// RUN SIM
	puts("About to start");
	// give time for xterm and picocom to start
	sleep(1);
	unsigned long cycles_run = 0;
	buffer[0] = '\0';
	while (1) {
		if (buffer[buffer_idx] != '\0' && cycles_run % 1000000 == 0) {
			avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT), buffer[buffer_idx]);
			buffer_idx++;
		}
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
