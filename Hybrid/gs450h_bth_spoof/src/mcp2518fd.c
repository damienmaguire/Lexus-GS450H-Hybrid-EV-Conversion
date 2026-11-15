/* mcp2518fd.c -- minimal classic-CAN driver for the MCP2518FD on SPI0.
 *
 * Board: Damien's AliExpress MCP2518FD module, 20 MHz crystal, 500 kbit/s.
 * Setup: CAN 2.0 mode, TX FIFO1 (16 deep), RX FIFO2 (16 deep), one filter
 * that accepts every ID, RX interrupt on the INT pin (GP15).
 * Register map: Microchip MCP251XXFD Family Reference Manual DS20005678.
 */
#include "mcp2518fd.h"

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define CMD_RESET  0x0
#define CMD_WRITE  0x2
#define CMD_READ   0x3

#define REG_C1CON       0x000
#define REG_C1NBTCFG    0x004
#define REG_C1INT       0x01C
#define REG_C1TREC      0x034
#define REG_C1FIFOCON1  0x05C
#define REG_C1FIFOSTA1  0x060
#define REG_C1FIFOUA1   0x064
#define REG_C1FLTCON0   0x1D0
#define REG_C1FLTOBJ0   0x1F0
#define REG_C1MASK0     0x1F4
#define REG_OSC         0xE00
#define REG_IOCON       0xE04
#define REG_ECCCON      0xE0C
#define REG_ECCSTA      0xE10
#define REG_DEVID       0xE14
#define RAM_START       0x400
#define RAM_SIZE        2048

#define CON_ISOCRCEN    (1u << 5)
#define CON_BRSDIS      (1u << 12)
#define CON_RTXAT       (1u << 16)
#define CON_STEF        (1u << 19)
#define CON_TXQEN       (1u << 20)
#define CON_OPMOD_SHIFT 21
#define CON_REQOP_SHIFT 24
#define OPMOD_CONFIG    4
#define OPMOD_CAN20     6

#define OSC_PLLEN       (1u << 0)
#define OSC_OSCRDY      (1u << 10)
#define OSC_PLLRDY      (1u << 11)

#define INT_RXIE        (1u << 17)
#define INT_RXOVIE      (1u << 27)
#define INT_IVMIE       (1u << 31)

#define FIFOCON_TFNRFNIE (1u << 0)
#define FIFOCON_RXOVIE   (1u << 3)
#define FIFOCON_TXEN     (1u << 7)
#define FIFOCON_FSIZE(n) ((uint32_t)((n) - 1) << 24)
#define FIFOCON_UINC     (1u << 8)
#define FIFOCON_TXREQ    (1u << 9)
#define FIFOCON_TXAT_3   (1u << 21)

#define FIFO_CON(n) ((uint16_t)(REG_C1FIFOCON1 + 12 * ((n) - 1)))
#define FIFO_STA(n) ((uint16_t)(REG_C1FIFOSTA1 + 12 * ((n) - 1)))
#define FIFO_UA(n)  ((uint16_t)(REG_C1FIFOUA1  + 12 * ((n) - 1)))
#define FIFOSTA_TFNRFNIF (1u << 0)
#define FIFOSTA_RXOVIF   (1u << 3)

/* TX FIFO1, RX FIFO2. TEF and TXQ are off, so FIFO1 starts at RAM 0x400. */
#define MCP_FIFO_TX  1
#define MCP_FIFO_RX  2
#define MCP_RX_DEPTH 16
#define MCP_TX_DEPTH 16

#define ECCCON_ECCEN (1u << 0)

static const mcp2518fd_cfg_t *g_cfg;
static uint32_t g_tx_seq;

/* Short delay around chip-select edges (50 CPU cycles). */
#define CS_GUARD() busy_wait_at_least_cycles(50)
static inline void cs_low(void)  { CS_GUARD(); gpio_put(g_cfg->pin_cs, 0); CS_GUARD(); }
static inline void cs_high(void) { CS_GUARD(); gpio_put(g_cfg->pin_cs, 1); CS_GUARD(); }

/* 2-byte SPI command header: 4-bit command + 12-bit address. */
static void hdr(uint8_t *h, uint8_t cmd, uint16_t addr) {
    h[0] = (uint8_t)((cmd << 4) | ((addr >> 8) & 0x0F));
    h[1] = (uint8_t)(addr & 0xFF);
}

/* SPI RESET command (puts the chip in config mode). */
static void mcp_reset_cmd(void) {
    uint8_t h[2] = {0, 0};
    cs_low();
    spi_write_blocking(g_cfg->spi, h, 2);
    cs_high();
}

/* Read len bytes from a register / RAM address. */
static void mcp_read(uint16_t addr, uint8_t *buf, size_t len) {
    uint8_t h[2];
    hdr(h, CMD_READ, addr);
    cs_low();
    spi_write_blocking(g_cfg->spi, h, 2);
    spi_read_blocking(g_cfg->spi, 0x00, buf, len);
    cs_high();
}

/* Write len bytes to a register / RAM address. */
static void mcp_write(uint16_t addr, const uint8_t *buf, size_t len) {
    uint8_t h[2];
    hdr(h, CMD_WRITE, addr);
    cs_low();
    spi_write_blocking(g_cfg->spi, h, 2);
    spi_write_blocking(g_cfg->spi, buf, len);
    cs_high();
}

/* Read a 32-bit little-endian register. */
static uint32_t rd32(uint16_t addr) {
    uint8_t b[4];
    mcp_read(addr, b, 4);
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

/* Write a 32-bit little-endian register. */
static void wr32(uint16_t addr, uint32_t v) {
    uint8_t b[4] = {
        (uint8_t)v, (uint8_t)(v >> 8),
        (uint8_t)(v >> 16), (uint8_t)(v >> 24)
    };
    mcp_write(addr, b, 4);
}

/* Write one byte of a register. */
static void wr8(uint16_t addr, uint8_t v) { mcp_write(addr, &v, 1); }

/* Wait up to timeout_ms for C1CON.OPMOD to report the given mode. */
static bool wait_opmod(uint8_t mode, uint32_t timeout_ms) {
    absolute_time_t until = make_timeout_time_ms(timeout_ms);
    do {
        uint8_t b2;
        mcp_read(REG_C1CON + 2, &b2, 1);
        if (((b2 >> 5) & 7) == mode) return true;
        sleep_us(100);
    } while (!time_reached(until));
    return false;
}

/* Zero the 2 KB message RAM (needed before ECC is enabled). */
static void ram_init(void) {
    uint8_t z[32];
    uint16_t a;
    memset(z, 0, sizeof z);
    for (a = 0; a < RAM_SIZE; a = (uint16_t)(a + sizeof z)) {
        mcp_write((uint16_t)(RAM_START + a), z, sizeof z);
    }
}

/* Text for an mcp_status_t, shown in the USB status line on CAN errors. */
const char *mcp2518fd_strerror(mcp_status_t s) {
    switch (s) {
    case MCP_OK:            return "ok";
    case MCP_ERR_SPI:       return "SPI check failed (no chip, wiring, or power)";
    case MCP_ERR_OSC:       return "oscillator not ready (wrong crystal, or not running)";
    case MCP_ERR_MODE:      return "mode change refused (bus stuck dominant?)";
    case MCP_ERR_BITTIMING: return "no bit timing for this oscillator / bitrate";
    case MCP_ERR_TX_FULL:   return "TX FIFO full";
    }
    return "?";
}

/* Prefer BRP = 1 (most time quanta) and a sample point of 80 %.
 * For 20 MHz / 500 kbit/s this is 40 TQ: TSEG1=31, TSEG2=8, SJW=8
 * (register NTSEG1=30, NTSEG2=7, NSJW=7, NBRP=0). */
bool mcp2518fd_calc_bittiming(uint32_t osc_hz, uint32_t bitrate, mcp_bittiming_t *bt) {
    uint32_t brp;
    if (bt == 0 || bitrate == 0 || osc_hz == 0) return false;
    memset(bt, 0, sizeof(*bt));
    for (brp = 1; brp <= 256; brp++) {
        uint32_t tq, sp_tq, tseg1, tseg2;
        if (osc_hz % (brp * bitrate)) continue;
        tq = osc_hz / (brp * bitrate);
        if (tq < 8 || tq > 1 + 256 + 128) continue;
        sp_tq = (tq * 80 + 99) / 100; /* 80 %, rounded up */
        tseg1 = sp_tq - 1;
        tseg2 = tq - sp_tq;
        if (tseg2 < 2 || tseg2 > 128 || tseg1 < 2 || tseg1 > 256) continue;
        bt->brp = brp;
        bt->tseg1 = tseg1;
        bt->tseg2 = tseg2;
        bt->sjw = tseg2;
        bt->tq_per_bit = tq;
        bt->sample_point_permille = (1000 * sp_tq) / tq;
        bt->nbrp = brp - 1;
        bt->ntseg1 = tseg1 - 1;
        bt->ntseg2 = tseg2 - 1;
        bt->nsjw = tseg2 - 1;
        return true;
    }
    return false;
}

/* Reset and configure the controller, then enter CAN 2.0 mode.
 * Called at start-up and again every 2 s while CAN is in error (main.c).
 * Leaves the clock-out pin at the reset default of /10 (2 MHz) instead of
 * a 20 MHz square wave on the header. */
mcp_status_t mcp2518fd_init(const mcp2518fd_cfg_t *cfg, mcp_bittiming_t *bt_out) {
    mcp_bittiming_t bt;
    uint32_t osc, con, nbt;
    absolute_time_t until;

    g_cfg = cfg;
    g_tx_seq = 0;

    if (!mcp2518fd_calc_bittiming(cfg->osc_hz, cfg->bitrate, &bt)) {
        return MCP_ERR_BITTIMING;
    }
    if (bt_out) *bt_out = bt;

    if (cfg->pin_stby >= 0) {
        gpio_init((uint)cfg->pin_stby);
        gpio_set_dir((uint)cfg->pin_stby, GPIO_OUT);
        gpio_put((uint)cfg->pin_stby, 1);
    }

    gpio_init(cfg->pin_int);
    gpio_set_dir(cfg->pin_int, GPIO_IN);
    gpio_pull_up(cfg->pin_int);

    gpio_init(cfg->pin_cs);
    gpio_set_dir(cfg->pin_cs, GPIO_OUT);
    gpio_put(cfg->pin_cs, 1);

    spi_init(cfg->spi, cfg->spi_hz);
    spi_set_format(cfg->spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(cfg->pin_sck, GPIO_FUNC_SPI);
    gpio_set_function(cfg->pin_mosi, GPIO_FUNC_SPI);
    gpio_set_function(cfg->pin_miso, GPIO_FUNC_SPI);
    gpio_pull_up(cfg->pin_miso);

    mcp_reset_cmd();
    sleep_ms(3); /* TOSCSTAB max 3 ms */

    until = make_timeout_time_ms(20);
    do {
        osc = rd32(REG_OSC);
        if (osc & OSC_OSCRDY) break;
        sleep_us(200);
    } while (!time_reached(until));

    con = rd32(REG_C1CON);
    if (((con >> CON_OPMOD_SHIFT) & 7) != OPMOD_CONFIG || con == 0xFFFFFFFFu) {
        return MCP_ERR_SPI;
    }
    if (!(osc & OSC_OSCRDY)) {
        return MCP_ERR_OSC;
    }

    /* 20 MHz crystal: leave PLL off, SYSCLK = 20 MHz. (PLL is for 4 MHz xtal.) */
    /* Keep the reset-default CLKODIV = /10 (2 MHz on CLKO) instead of a
     * 20 MHz square wave on the header's CLK pin. SYSCLK unchanged. */
    wr32(REG_OSC, 0x60);

    /* CiNBTCFG: BRP[31:24] TSEG1[23:16] TSEG2[15:8] SJW[6:0], each field is
     * (real TQ count - 1). */
    nbt = (bt.nbrp << 24) | (bt.ntseg1 << 16) | (bt.ntseg2 << 8) | bt.nsjw;
    wr32(REG_C1NBTCFG, nbt);
    if (rd32(REG_C1NBTCFG) != nbt) return MCP_ERR_SPI;

    /* INT0/INT1 unused on this header: drive as GPIOs (byte-wise IOCON write). */
    wr8(REG_IOCON + 0, 0x00);

    /* Zero RAM, then enable ECC and clear sticky ECC status. */
    ram_init();
    wr32(REG_ECCCON, ECCCON_ECCEN);
    wr32(REG_ECCSTA, 0);

    /* No TX event FIFO, no TX queue: FIFO1 sits at RAM 0x400.
     * Restricted retransmissions (3 tries). Disable bit-rate switching. */
    con = rd32(REG_C1CON);
    con &= ~(CON_STEF | CON_TXQEN);
    con |= CON_RTXAT | CON_ISOCRCEN | CON_BRSDIS;
    con = (con & ~(7u << CON_REQOP_SHIFT)) | ((uint32_t)OPMOD_CONFIG << CON_REQOP_SHIFT);
    wr32(REG_C1CON, con);

    wr32(FIFO_CON(MCP_FIFO_TX),
         FIFOCON_TXEN | FIFOCON_TXAT_3 | FIFOCON_FSIZE(MCP_TX_DEPTH));
    wr32(FIFO_CON(MCP_FIFO_RX),
         FIFOCON_TFNRFNIE | FIFOCON_RXOVIE | FIFOCON_FSIZE(MCP_RX_DEPTH));

    /* Filter 0 -> FIFO2, mask 0, MIDE = 0: accept all standard and extended IDs. */
    wr32(REG_C1FLTOBJ0, 0);
    wr32(REG_C1MASK0, 0);
    wr8(REG_C1FLTCON0, (uint8_t)(0x80 | MCP_FIFO_RX));

    wr32(REG_C1INT, INT_RXIE | INT_RXOVIE);

    if (cfg->pin_stby >= 0) {
        gpio_put((uint)cfg->pin_stby, 0);
        sleep_us(50);
    }

    wr8(REG_C1CON + 3, OPMOD_CAN20); /* REQOP = CAN 2.0 (mode 6) */
    if (!wait_opmod(OPMOD_CAN20, 20)) return MCP_ERR_MODE;
    return MCP_OK;
}

/* Queue one frame in TX FIFO1. Returns MCP_ERR_TX_FULL if there is no room. */
mcp_status_t mcp2518fd_send(const mcp_frame_t *f) {
    uint8_t sta, dlc;
    uint32_t ua, t0, t1;
    uint16_t addr;
    uint8_t obj[16];

    if (f == 0) return MCP_ERR_SPI;
    dlc = f->dlc;
    if (dlc > 8) dlc = 8;

    mcp_read(FIFO_STA(MCP_FIFO_TX), &sta, 1);
    if (!(sta & FIFOSTA_TFNRFNIF)) return MCP_ERR_TX_FULL;

    ua = rd32(FIFO_UA(MCP_FIFO_TX));
    addr = (uint16_t)(RAM_START + (ua & 0xFFF));

    memset(obj, 0, sizeof obj);
    if (f->ide) {
        t0 = ((f->id >> 18) & 0x7FFu) | ((f->id & 0x3FFFFu) << 11);
        t1 = (1u << 4) | (dlc & 0xF);
    } else {
        t0 = f->id & 0x7FFu;
        t1 = (dlc & 0xF);
    }
    if (f->rtr) t1 |= (1u << 5);
    t1 |= ((g_tx_seq++ & 0x7FFFFFu) << 9);
    obj[0] = (uint8_t)t0;
    obj[1] = (uint8_t)(t0 >> 8);
    obj[2] = (uint8_t)(t0 >> 16);
    obj[3] = (uint8_t)(t0 >> 24);
    obj[4] = (uint8_t)t1;
    obj[5] = (uint8_t)(t1 >> 8);
    obj[6] = (uint8_t)(t1 >> 16);
    obj[7] = (uint8_t)(t1 >> 24);
    if (!f->rtr && dlc > 0) memcpy(&obj[8], f->data, dlc);
    mcp_write(addr, obj, 16);

    wr8((uint16_t)(FIFO_CON(MCP_FIFO_TX) + 1),
        (uint8_t)((FIFOCON_UINC | FIFOCON_TXREQ) >> 8));
    return MCP_OK;
}

/* Read one frame from RX FIFO2. Returns 1 = frame read, 0 = FIFO empty,
 * negative = SPI error. *overflow is set (and cleared in the chip) if the
 * FIFO overflowed. */
int mcp2518fd_recv(mcp_frame_t *f, bool *overflow) {
    uint8_t b[8], o[16], r1, len;
    uint32_t ua, r0;
    uint16_t addr;

    if (overflow) *overflow = false;
    mcp_read(FIFO_STA(MCP_FIFO_RX), b, 8);
    ua = (uint32_t)b[4] | ((uint32_t)b[5] << 8) |
         ((uint32_t)b[6] << 16) | ((uint32_t)b[7] << 24);
    if (ua == 0xFFFFFFFFu) return MCP_ERR_SPI;
    if (b[0] & FIFOSTA_RXOVIF) {
        if (overflow) *overflow = true;
        wr8(FIFO_STA(MCP_FIFO_RX), 0x00);
    }
    if (!(b[0] & FIFOSTA_TFNRFNIF)) return 0;

    addr = (uint16_t)(RAM_START + (ua & 0xFFF));
    mcp_read(addr, o, 16);
    r0 = (uint32_t)o[0] | ((uint32_t)o[1] << 8) |
         ((uint32_t)o[2] << 16) | ((uint32_t)o[3] << 24);
    r1 = o[4];
    f->ide = (r1 >> 4) & 1;
    f->rtr = (r1 >> 5) & 1;
    f->dlc = r1 & 0x0F;
    f->id = f->ide ? (((r0 & 0x7FFu) << 18) | ((r0 >> 11) & 0x3FFFFu))
                   : (r0 & 0x7FFu);
    len = f->dlc;
    if (len > 8) len = 8;
    if (f->rtr) len = 0;
    memset(f->data, 0, 8);
    if (len) memcpy(f->data, &o[8], len);

    wr8((uint16_t)(FIFO_CON(MCP_FIFO_RX) + 1), (uint8_t)(FIFOCON_UINC >> 8));
    return 1;
}

/* Error counters, bus-off / passive / warning flags, operating mode and
 * whether a TX is still pending. */
mcp_status_t mcp2518fd_read_status(mcp_status_info_t *st) {
    uint32_t trec, con;
    uint8_t conb[2];
    if (st == 0) return MCP_ERR_SPI;
    memset(st, 0, sizeof(*st));
    trec = rd32(REG_C1TREC);
    if (trec == 0xFFFFFFFFu) return MCP_ERR_SPI;
    st->rec = (uint8_t)trec;
    st->tec = (uint8_t)(trec >> 8);
    st->tx_warn    = (trec >> 18) & 1;
    st->rx_warn    = (trec >> 17) & 1;
    st->rx_passive = (trec >> 19) & 1;
    st->tx_passive = (trec >> 20) & 1;
    st->bus_off    = (trec >> 21) & 1;
    con = rd32(REG_C1CON);
    st->opmod = (uint8_t)((con >> CON_OPMOD_SHIFT) & 7);
    mcp_read(FIFO_CON(MCP_FIFO_TX), conb, 2);
    st->tx_pending = (conb[1] & (uint8_t)(FIFOCON_TXREQ >> 8)) != 0;
    return MCP_OK;
}

/* True while the INT pin is low (RX FIFO not empty). */
bool mcp2518fd_int_asserted(void) {
    if (!g_cfg) return false;
    return gpio_get(g_cfg->pin_int) == 0;
}
