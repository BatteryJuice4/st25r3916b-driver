/*
 * st25r3916.h — ST25R3916B NFC front-end driver (chip logic, transport-injected).
 *
 * An independent C driver written from the ST datasheet DS13541 Rev 8 (see st25r3916_regs.h).
 * It does not use or derive from ST's RFAL middleware. It is MCU-independent: the host passes
 * a bus with one SPI transaction primitive and a microsecond delay, so the same code builds
 * and is unit-tested on a PC (test/test_st25r3916.c) and runs on any microcontroller once you
 * implement that bus for your SPI peripheral and the chip's pins.
 *
 * STATUS / UNTESTED ON HARDWARE: this is the register/command/SPI foundation plus device
 * identification. The register and command values are transcribed from the datasheet. The
 * higher-level RF flows (polling/anticollision, transceive, card emulation, wake-up mode) are
 * not implemented yet and, when added, must be verified against the datasheet and a real
 * chip. See README.md.
 */
#ifndef PDA_DRV_ST25R3916_H
#define PDA_DRV_ST25R3916_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * One full-duplex SPI transaction, framed by chip-select inside the callback: the board
 * drives BSS (CS) low, clocks `n` bytes out of `tx` while reading `n` bytes into `rx`
 * (either may be NULL for a one-way transfer), then drives BSS high. SPI is mode 1, MSB
 * first (DS13541 §4.3.3). `delay_us` waits at least `us` microseconds.
 */
typedef struct {
    void *ctx;
    void (*txn)(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n);
    void (*delay_us)(void *ctx, unsigned us);
} st25r_bus_t;

typedef struct {
    const st25r_bus_t *bus;
    uint8_t ic_type; /* filled by st25r3916_probe() */
    uint8_t ic_rev;
} st25r_t;

/* ---- Register and command access (DS13541 §4.3.3, §4.4) -------------------------------- */

/* Issue a direct command (one of ST25R_CMD_*). */
void st25r3916_cmd(st25r_t *d, uint8_t command);
/* Space-A register read/write (addr 0x00..0x3F). */
uint8_t st25r3916_read_reg(st25r_t *d, uint8_t addr);
void st25r3916_write_reg(st25r_t *d, uint8_t addr, uint8_t val);
/* Read `n` consecutive space-A registers from `addr` using SPI address auto-increment. */
void st25r3916_read_regs(st25r_t *d, uint8_t addr, uint8_t *out, size_t n);
/* Read-modify-write: out = (reg & ~mask) | (val & mask). */
void st25r3916_modify_reg(st25r_t *d, uint8_t addr, uint8_t mask, uint8_t val);
/* Space-B register read/write (prefixed with 0xFB; DS13541 Table 19). */
uint8_t st25r3916_read_regb(st25r_t *d, uint8_t addr);
void st25r3916_write_regb(st25r_t *d, uint8_t addr, uint8_t val);

/* ---- FIFO (DS13541 §4.3.3 "Loading/Reading … FIFO") ------------------------------------ */
void st25r3916_write_fifo(st25r_t *d, const uint8_t *data, size_t n); /* n: 1..512 */
void st25r3916_read_fifo(st25r_t *d, uint8_t *out, size_t n);
/* Bytes currently in the FIFO, from FIFO status registers 1/2 (0x1E/0x1F). */
int st25r3916_fifo_count(st25r_t *d);

/* ---- Interrupts ------------------------------------------------------------------------ */
/* Read and clear the four interrupt registers (0x1A..0x1D) in one burst; returns them packed
 * little-endian: bits 7:0 main, 15:8 timer/NFC, 23:16 error/wake-up, 31:24 passive target
 * (use the ST25R_IRQ_* macros / the ST25R_IRQ_MAIN() etc. selectors). */
uint32_t st25r3916_get_interrupts(st25r_t *d);
/* Write the four interrupt-mask registers (0x16..0x19) from the same packed layout; a 1 bit
 * masks (disables) that interrupt. */
void st25r3916_set_interrupt_mask(st25r_t *d, uint32_t mask);

/* ---- Identity / bring-up -------------------------------------------------------------- */
/* Read the IC identity register (0x3F) into d->ic_type / d->ic_rev. Returns true if the part
 * reads back as an ST25R3916B (ic_type == 0x06). */
bool st25r3916_probe(st25r_t *d);

#endif
