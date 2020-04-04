
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>

#include <simavr/sim_irq.h>

#include "sd.h"

#define true 1
#define false 0

#define BLOCK_SIZE 512

#ifdef SD_DEBUG
#include <stdio.h>
#define DEBUG(msg, ...) fprintf(stderr, "SD_DEBUG: " msg "\n", ##__VA_ARGS__)
#else
#define DEBUG(msg, ...)
#endif

// shamelessly copied straight from the Linux Kernel. Luckily I love the GPL
unsigned short const crc16_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

const unsigned char crc7_be_syndrome_table[256] = {
	0x00, 0x12, 0x24, 0x36, 0x48, 0x5a, 0x6c, 0x7e,
	0x90, 0x82, 0xb4, 0xa6, 0xd8, 0xca, 0xfc, 0xee,
	0x32, 0x20, 0x16, 0x04, 0x7a, 0x68, 0x5e, 0x4c,
	0xa2, 0xb0, 0x86, 0x94, 0xea, 0xf8, 0xce, 0xdc,
	0x64, 0x76, 0x40, 0x52, 0x2c, 0x3e, 0x08, 0x1a,
	0xf4, 0xe6, 0xd0, 0xc2, 0xbc, 0xae, 0x98, 0x8a,
	0x56, 0x44, 0x72, 0x60, 0x1e, 0x0c, 0x3a, 0x28,
	0xc6, 0xd4, 0xe2, 0xf0, 0x8e, 0x9c, 0xaa, 0xb8,
	0xc8, 0xda, 0xec, 0xfe, 0x80, 0x92, 0xa4, 0xb6,
	0x58, 0x4a, 0x7c, 0x6e, 0x10, 0x02, 0x34, 0x26,
	0xfa, 0xe8, 0xde, 0xcc, 0xb2, 0xa0, 0x96, 0x84,
	0x6a, 0x78, 0x4e, 0x5c, 0x22, 0x30, 0x06, 0x14,
	0xac, 0xbe, 0x88, 0x9a, 0xe4, 0xf6, 0xc0, 0xd2,
	0x3c, 0x2e, 0x18, 0x0a, 0x74, 0x66, 0x50, 0x42,
	0x9e, 0x8c, 0xba, 0xa8, 0xd6, 0xc4, 0xf2, 0xe0,
	0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x70,
	0x82, 0x90, 0xa6, 0xb4, 0xca, 0xd8, 0xee, 0xfc,
	0x12, 0x00, 0x36, 0x24, 0x5a, 0x48, 0x7e, 0x6c,
	0xb0, 0xa2, 0x94, 0x86, 0xf8, 0xea, 0xdc, 0xce,
	0x20, 0x32, 0x04, 0x16, 0x68, 0x7a, 0x4c, 0x5e,
	0xe6, 0xf4, 0xc2, 0xd0, 0xae, 0xbc, 0x8a, 0x98,
	0x76, 0x64, 0x52, 0x40, 0x3e, 0x2c, 0x1a, 0x08,
	0xd4, 0xc6, 0xf0, 0xe2, 0x9c, 0x8e, 0xb8, 0xaa,
	0x44, 0x56, 0x60, 0x72, 0x0c, 0x1e, 0x28, 0x3a,
	0x4a, 0x58, 0x6e, 0x7c, 0x02, 0x10, 0x26, 0x34,
	0xda, 0xc8, 0xfe, 0xec, 0x92, 0x80, 0xb6, 0xa4,
	0x78, 0x6a, 0x5c, 0x4e, 0x30, 0x22, 0x14, 0x06,
	0xe8, 0xfa, 0xcc, 0xde, 0xa0, 0xb2, 0x84, 0x96,
	0x2e, 0x3c, 0x0a, 0x18, 0x66, 0x74, 0x42, 0x50,
	0xbe, 0xac, 0x9a, 0x88, 0xf6, 0xe4, 0xd2, 0xc0,
	0x1c, 0x0e, 0x38, 0x2a, 0x54, 0x46, 0x70, 0x62,
	0x8c, 0x9e, 0xa8, 0xba, 0xc4, 0xd6, 0xe0, 0xf2
};

static void crc16_byte(unsigned short *crc, unsigned char data) {
	*crc = ((*crc) >> 8) ^ crc16_table[((*crc) ^ data) & 0xff];
}

// the good thing about open source is that I'm allowed to move the bracket onto
// the same line as the function signature, as God intended.
static void crc7_byte(unsigned char *crc, unsigned char data) {
	*crc = crc7_be_syndrome_table[(*crc) ^ data];
}

const char *irq_names[] = {
	[SD_IRQ_MOSI] = "SD_IRQ_MOSI",
	[SD_IRQ_MISO] = "SD_IRQ_MISO",
	[SD_IRQ_CS] = "SD_IRQ_CS"
};

// Reset the card. Like a hard power cycle.
void sd_reset(sd_t *sd) {
	DEBUG("Reset!");
	// do NOT reset sd->cs -- that's controlled by interrupts

	sd->enforce_crc = false;
	// also, do not need to reset read/write related variables -- they will
	// be reset when entering a read- or write-related mode.
	sd->cmd_idx = 0;
	sd->send_idx = 0;
	sd->send_len = 0;
	sd->state = SD_STATE_BOOT;
}

// reset the card and send an empty byte over SPI. For critical errors that
// occur while processing a byte (i.e, most of them).
static void error_reset(sd_t *sd) {
	sd_reset(sd);
	avr_raise_irq(sd->irq + SD_IRQ_MISO, 0x00);
}

static void enqueue_r1_inner(sd_t *sd, unsigned char byte) {
	sd->send[0] = byte;
	sd->send_len = 1;
	sd->state = SD_STATE_CMD_RESPONSE;
}

#define enqueue_r1(sd) enqueue_r1_inner(sd, 0x00)
#define enqueue_idle_r1(sd) enqueue_r1_inner(sd, 0x01)
#define enqueue_illegal_command_r1(sd) enqueue_r1_inner(sd, 0x00)
#define enqueue_data_response(sd) enqueue_r1_inner(sd, 0x05)
#define enqueue_crc_error_data_response(sd) enqueue_r1_inner(sd, 0x0b);
#define enqueue_illegal_command(sd) enqueue_r1_inner(sd, 0x04)
#define enqueue_address_error(sd) enqueue_r1_inner(sd, 0x20)

static void enqueue_r2(sd_t *sd) {
	sd->send[0] = 0b00000000;
	sd->send[1] = 0b00000000;
	sd->send_len = 2;
	sd->state = SD_STATE_CMD_RESPONSE;
}

static void enqueue_r3(sd_t *sd) {
	enqueue_r1(sd);
	// set OCR as to emulate a standard-speed SD card that supports any
	// voltage.
	sd->send[1] = 0b10000001;
	sd->send[2] = 0b11111111;
	sd->send[3] = 0b00000000;
	sd->send[4] = 0b00000000;
	sd->send_len = 5;
	sd->state = SD_STATE_CMD_RESPONSE;
}

// TODO: verify that these crcs are actually correct! The Arduino library does
// *not* check them!
static void enqueue_crc16(sd_t *sd) {
	sd->send[0] = sd->crc16 >> 8;
	sd->send[1] = sd->crc16 & 0xFF;
	sd->send_len = 2;
	sd->state = SD_STATE_CMD_RESPONSE;
}

// Called after a complete command was received. Analyzes the command and places
// an appropriate response in the output buffer. Also changes the SD card object
// as necessary (eg, changing state).
static void enqueue_response(sd_t *sd) {
	// TODO: verify CRC, basic format of received message.
	/* if (sd->cmd[0] & (1 << 7) || sd->cmd[0] ^ (1 << 6)) */

	sd->after_send_state = sd->state;

	sd->send_idx = 0;
	unsigned char command_index = sd->cmd[0] & 0b00111111;
	uint32_t command_arg =
		(sd->cmd[1] << 24) +
		(sd->cmd[2] << 16) +
		(sd->cmd[3] <<  8) +
		(sd->cmd[4] <<  0);
	DEBUG("Received command %d with argument %lu",
	      command_index, (unsigned long)command_arg);

	// only valid command in boot state is reset.
	if (sd->state == SD_STATE_BOOT && command_index != 0) {
		// unlike the other state-based exceptions below
		return enqueue_illegal_command(sd);
	}

	// this is probably incorrect -- many commands other than CMD55 and
	// ACMD41 are likely supposed to work before ACMD41.
	if (sd->state == SD_STATE_SPI) {
		if (command_index == 55) {
			DEBUG("CMD55 from SPI mode");
			sd->after_send_state = SD_STATE_SPI_ACMD;
			return enqueue_idle_r1(sd);
		}
		return enqueue_illegal_command(sd);
	}

	if (sd->state == SD_STATE_SPI_ACMD) {
		if (command_index == 41) {
			DEBUG("ACMD41 from SPI mode");
			sd->after_send_state = SD_STATE_IDLE;
			return enqueue_r1(sd);
		}
		return enqueue_illegal_command(sd);
	}

	if (sd->state != SD_STATE_IDLE_ACMD) {
		DEBUG("Handling as CMD");
		switch (command_index) {
		case 0:
			sd_reset(sd);
			sd->after_send_state = SD_STATE_SPI;
			enqueue_idle_r1(sd);
			break;
		case 13:
			// SEND_STATUS
			enqueue_r2(sd);
			break;
		case 17:
		case 18:
			DEBUG("Read block beginning at byte %d", command_arg);
			if (command_arg > sd->capacity - BLOCK_SIZE) {
				DEBUG("Illegal start byte!");
				enqueue_address_error(sd);
				break;
			}
			sd->head = sd->mass + command_arg;
			sd->bytes_xfrd = 0;
			sd->crc16 = 0xFFFF;
			sd->multiple_block = command_index == 18;
			sd->after_send_state = SD_STATE_READ_BLOCK;
			sd->send[0] = 0x00;
			sd->send[1] = 0xFE;
			sd->send_len = 2;
			sd->state = SD_STATE_CMD_RESPONSE;
			break;
		case 24:
		case 25:
			DEBUG("Write block beginning at byte %d", command_arg);
			if (command_arg > sd->capacity - BLOCK_SIZE) {
				DEBUG("Illegal start byte!\n");
				enqueue_address_error(sd);
				break;
			}
			sd->after_send_state = SD_STATE_WRITE_STBT;
			sd->head = sd->mass + command_arg;
			sd->bytes_xfrd = 0;
			sd->crc16 = 0xFFFF;
			sd->multiple_block = command_index == 25;
			enqueue_r1(sd);
			break;

		case 55:
			enqueue_r1(sd);
			sd->after_send_state = SD_STATE_IDLE_ACMD;
			break;
		case 58:
			enqueue_r3(sd);
			break;
		default:
			DEBUG("Unknown/illegal command");
			enqueue_illegal_command(sd);
		}
	} else {
		DEBUG("Handling as ACMD");
		sd->state = SD_STATE_IDLE;
		switch (command_index) {
		default:
			enqueue_illegal_command(sd);
		}
	}

}

// return the next byte to be sent over SPI
static unsigned char send_byte(sd_t *sd) {
	switch (sd->state) { 
	case SD_STATE_READ_BLOCK:
//		DEBUG("Sending byte %d", sd->bytes_xfrd);
		crc16_byte(&sd->crc16, *(unsigned char*)(sd->head));
		if (++(sd->bytes_xfrd) == BLOCK_SIZE) {
			DEBUG("Block fully read and transmitted");
			enqueue_crc16(sd);
		}
		return *(unsigned char *)(sd->head++);
	case SD_STATE_WRITE_CRC:
		return 0x05;
	case SD_STATE_CMD_RESPONSE:;
		unsigned char result = sd->send[sd->send_idx++];
		if (sd->send_idx == sd->send_len) {
			sd->state = sd->after_send_state;
			sd->after_send_state = SD_STATE_IDLE;
			sd->send_idx = 0;
		}
		return result;
	}

	return 0xFF;
}

// update the sd object based on a byte received over SPI
static void accept_byte(sd_t *sd, unsigned char byte) {
	switch (sd->state) {
	case SD_STATE_WRITE_STBT:
		if (byte == 0xFE) {
			DEBUG("Received write start block token.");
			sd->state = SD_STATE_WRITE_LISTEN;
		}
		// TODO: (this is a larger todo) We should be listening for a
		// reset command even now, right?
		break;
	case SD_STATE_WRITE_LISTEN:
		*(unsigned char *)(sd->head++) = byte;
		crc16_byte(&sd->crc16, byte);
		if (++(sd->bytes_xfrd) == BLOCK_SIZE) {
			DEBUG("Entire block received -- Receiving CRC");
			sd->crc16_fst = true;
			sd->state = SD_STATE_WRITE_CRC;
		}
		break;
	case SD_STATE_WRITE_CRC:;
		unsigned char expected = sd->crc16_fst ?
			sd->crc16 & 0xFF
			: sd->crc16 >> 8;
		DEBUG("CRC Byte: Expected %d, got %d.", expected, byte);
		sd->after_send_state = SD_STATE_IDLE;
		if (expected == byte || !sd->enforce_crc) {
			if (!sd->crc16_fst) {
				enqueue_data_response(sd);
			}
		} else {
			// this is wrong -- it might send the response after
			// reading only the first byte of the CRC -- but the
			// Arduino library doesn't enforce CRCs anyway!
			enqueue_crc_error_data_response(sd);
		}
		sd->crc16_fst = false;
		break;
	// modes where we should, theoretically, never receive a
	// command. Treat any byte as an error.
	// TODO: remove the next few once we have multi-block read support. And
	// verify that it's actually illegal for a command to be sent here.
	case SD_STATE_CMD_RESPONSE:
		if (byte ^ 0xFF) {
			DEBUG("Received a command while sending");
			error_reset(sd);
		}
		break;
	// In all other modes, it's valid to receive a normal command.
	default:
		if (byte ^ 0xFF || sd->cmd_idx) {
			sd->cmd[sd->cmd_idx] = byte;
			sd->cmd_idx++;

			if (sd->cmd_idx == COMMAND_LENGTH) {
				sd->cmd_idx = 0;
				enqueue_response(sd);
			}
		}
	}
}

static void spi_hook(avr_irq_t *irq, uint32_t value, void *param) {
	sd_t *sd = (sd_t *)param;

	// ignore unless chip selected
	if (!sd->cs) return;

	avr_raise_irq(sd->irq + SD_IRQ_MISO, send_byte(sd));
	accept_byte(sd, value & 0xFF);
}

static void cs_hook(avr_irq_t *irq, uint32_t value, void *param) {
	sd_t *sd = (sd_t *)param;
	sd->cs = !value;

	if (value) {
		// TODO: we should probably do something (reset?) if we're
		// deselected in certain states.
		DEBUG("Chip deselected");
	} else {
		DEBUG("Chip selected");
	}
}

int sd_init(sd_t *sd, avr_t *avr, char *image_path) {
	int fd;
	if ((fd = open(image_path, O_RDWR)) == -1) {
		// TODO: handle errors
		close(fd);
		return -1;
	}
	sd->capacity = (size_t) lseek(fd, 0, SEEK_END);
	sd->mass = mmap(NULL, sd->capacity, PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, 0);
	if (sd->mass == MAP_FAILED) return -1;
	close(fd);

	sd->irq = avr_alloc_irq(&avr->irq_pool, 0, SD_IRQ_LEN, irq_names);
	avr_irq_register_notify(sd->irq + SD_IRQ_MOSI, &spi_hook, sd);
	avr_irq_register_notify(sd->irq + SD_IRQ_CS, &cs_hook, sd);

	// because of this, it's important that the SD card is initialized and
	// connected before the first avr_run()
	sd->cs = false;
	sd_reset(sd);

	return 0;
}

void sd_free(sd_t *sd) {
	// implicitly performs msync()
	munmap(sd->mass, sd->capacity);
	avr_free_irq(sd->irq, SD_IRQ_LEN);
}
