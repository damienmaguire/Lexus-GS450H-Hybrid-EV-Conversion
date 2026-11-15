#ifndef MCP2518FD_H
#define MCP2518FD_H

/* mcp2518fd.h -- small self-contained classic-CAN driver for the
 * MCP2518FD (and MCP2517FD). Written for this project, not a vendored
 * Microchip library. Register map: MCP251XXFD Family Reference Manual
 * DS20005678.
 */
#include <stdbool.h>
#include <stdint.h>
#include "hardware/spi.h"

/* Return codes. */
typedef enum {
    MCP_OK = 0,
    MCP_ERR_SPI = -1,
    MCP_ERR_OSC = -2,
    MCP_ERR_MODE = -3,
    MCP_ERR_BITTIMING = -4,
    MCP_ERR_TX_FULL = -5
} mcp_status_t;

/* Pins and clocks (set in main.c core1_can()). */
typedef struct {
    spi_inst_t *spi;
    uint pin_miso, pin_mosi, pin_sck, pin_cs, pin_int;
    int pin_stby;       /* GPIO, or -1 if STBY is not on the board header */
    uint32_t spi_hz;
    uint32_t osc_hz;    /* crystal, 20 MHz on Damien's AliExpress board */
    uint32_t bitrate;   /* 500000 */
} mcp2518fd_cfg_t;

/* Nominal bit timing worked out by mcp2518fd_calc_bittiming(). */
typedef struct {
    uint32_t brp, tseg1, tseg2, sjw; /* real TQ counts (register + 1) */
    uint32_t tq_per_bit;
    uint32_t sample_point_permille;
    uint32_t nbrp, ntseg1, ntseg2, nsjw; /* raw register fields */
} mcp_bittiming_t;

/* One classic CAN frame. */
typedef struct {
    uint32_t id;
    bool ide;
    bool rtr;
    uint8_t dlc;
    uint8_t data[8];
} mcp_frame_t;

/* Controller status, from mcp2518fd_read_status(). */
typedef struct {
    uint8_t tec, rec;
    bool bus_off, tx_passive, rx_passive, tx_warn, rx_warn;
    uint8_t opmod;
    bool tx_pending;
} mcp_status_info_t;

/* Bit timing for osc_hz / bitrate: smallest prescaler, 80 % sample point.
 * 20 MHz / 500 kbit/s gives 40 TQ (TSEG1 31, TSEG2 8, SJW 8). */
bool mcp2518fd_calc_bittiming(uint32_t osc_hz, uint32_t bitrate, mcp_bittiming_t *bt);
/* Reset, configure and start the controller (CAN 2.0 mode). */
mcp_status_t mcp2518fd_init(const mcp2518fd_cfg_t *cfg, mcp_bittiming_t *bt_out);
/* Queue one frame for transmit. */
mcp_status_t mcp2518fd_send(const mcp_frame_t *f);
/* 1 = frame read, 0 = RX empty, negative = error. */
int mcp2518fd_recv(mcp_frame_t *f, bool *overflow);
/* Read error counters and mode. */
mcp_status_t mcp2518fd_read_status(mcp_status_info_t *st);
/* True while INT is low (frames waiting). */
bool mcp2518fd_int_asserted(void);
/* Human-readable text for a return code. */
const char *mcp2518fd_strerror(mcp_status_t s);

#endif
