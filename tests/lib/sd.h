#ifndef SD_H
#define SD_H

#include <sys/types.h>
#include <stddef.h>
#include <stdbool.h>

#include <simavr/sim_avr.h>

// Maximum number of bytes in a command or response
#define COMMAND_LENGTH 6

enum {
	SD_IRQ_MOSI,
	SD_IRQ_MISO,
	SD_IRQ_CS,
	SD_IRQ_LEN,		// utility; simply tells us how many IRQs there
				// are to simplify avr_alloc_ir
};

enum {
	SD_STATE_BOOT,		// Card just started up.
	SD_STATE_SPI,		// Card is in SPI mode (after a CMD0 is received
				// in SD_STATE_BOOT)
	SD_STATE_SPI_ACMD,      // An ACMD command has been selected, but we
				// aren't fully initialized yet, so shouldn't
				// accept most commands yet.
	SD_STATE_IDLE,		// This might conflict with the formal
				// definition of idle state. Here, it means the
				// SD card is waiting for a normal command.
	SD_STATE_IDLE_ACMD,	// Last command was CMD55, waiting for next one.
	SD_STATE_CMD_RESPONSE,	// In the middle of sending a normal response.
	SD_STATE_READ_BLOCK,	// In the middle of sending a data block.
	SD_STATE_WRITE_STBT,	// Waiting for the "start block token" that
				// precedes each block.
	SD_STATE_WRITE_LISTEN,	// Receiving a block
	SD_STATE_WRITE_CRC,	// Receive CRC after a block
};

#define SD_IDLE_P(st) \
	((st) == SD_STATE_IDLE || (st) == SD_STATE_IDLE_ACMD)
#define SD_READ_P(st) \
	((st) == SD_STATE_READ_RESPONSE || \
	 (st) == SD_STATE_READ_BLOCK || \
	 (st) == SD_STATE_READ_CRC)
#define SD_WRITE_P(st) \
	((st) == SD_STATE_WRITE_RESPONSE || \
	 (st) == SD_STATE_WRITE_STBT || \
	 (st) == SD_STATE_WRITE_LISTEN || \
	 (st) == SD_STATE_WRITE_DRES)

typedef struct sd_t {
	avr_irq_t *irq;

	int state;
	int after_send_state;	// which state to change to after sending the
				// current command response.
	bool cs;		// True when CS is low; this is logical.
	bool enforce_crc;

	unsigned char cmd[COMMAND_LENGTH]; // Command read in progress
	unsigned char cmd_idx;	// Which byte of command we will read next
	unsigned char send[COMMAND_LENGTH]; // Command write in progress
	unsigned char send_idx;
	unsigned char send_len; // The total number of bytes to send

	// TODO: mass and head as unsigned char* instead?
	void *mass;		// SD card block data, from mmap
	size_t capacity;
	unsigned short crc16;	// In-progress crc
	bool crc16_fst;		// If true, we are receiving the first byte of
				// the write CRC.
	void *head;		// read/write "head", points to somewhere in
				// mass
	int bytes_xfrd;		// Bytes of the block already transferred.
	bool multiple_block;	// True for CMD{18,25}, false for CMD{17,24}
} sd_t;

// Initialize an SD card. You allocate the structure memory. The image_path is
// the file the SD card maps to. *You* must create this file before calling
// sd_init. Its size is the size of the SD card. A blank file can be created
// with `truncate -s 2G file.img`, for example. Creating a "sparse" file in this
// manner will use negligible disk space. Check that errno = 0 after running!
// @return 0 on success, nonzero on failure. The only failure modes are
// IO-related, so check errno.
int sd_init(sd_t *sd, avr_t *avr, char *image_path);

// Free the memory used by the fields of an SD card structure. does not free the
// structure itself.
void sd_free(sd_t *);

// perform a power cycle
void sd_reset(sd_t *sd);

#endif // SD_H
