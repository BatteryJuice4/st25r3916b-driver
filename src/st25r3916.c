/*
 * st25r3916.c — ST25R3916B driver core (transport-injected). See st25r3916.h.
 * Values from DS13541 Rev 8; see st25r3916_regs.h. No dynamic allocation; safe from an ISR
 * if your bus callback is.
 */
#include "st25r3916.h"

#include <string.h>

#include "st25r3916_regs.h"

#define ST25R_REG_BURST_MAX 16 /* registers per auto-increment access we support */

static void txn(st25r_t *d, const uint8_t *tx, uint8_t *rx, size_t n)
{
    d->bus->txn(d->bus->ctx, tx, rx, n);
}

void st25r3916_cmd(st25r_t *d, uint8_t command)
{
    uint8_t b = command; /* the command byte already carries the '11' prefix (Table 14) */
    txn(d, &b, NULL, 1);
}

uint8_t st25r3916_read_reg(st25r_t *d, uint8_t addr)
{
    uint8_t out = 0;
    st25r3916_read_regs(d, addr, &out, 1);
    return out;
}

void st25r3916_read_regs(st25r_t *d, uint8_t addr, uint8_t *out, size_t n)
{
    if (n == 0 || n > ST25R_REG_BURST_MAX) {
        return;
    }
    uint8_t tx[1 + ST25R_REG_BURST_MAX] = {ST25R_SPI_REG_READ(addr)};
    uint8_t rx[1 + ST25R_REG_BURST_MAX];
    txn(d, tx, rx, n + 1);
    memcpy(out, rx + 1, n); /* first returned byte is clocked out during the mode byte */
}

void st25r3916_write_reg(st25r_t *d, uint8_t addr, uint8_t val)
{
    uint8_t tx[2] = {ST25R_SPI_REG_WRITE(addr), val};
    txn(d, tx, NULL, 2);
}

void st25r3916_modify_reg(st25r_t *d, uint8_t addr, uint8_t mask, uint8_t val)
{
    uint8_t cur = st25r3916_read_reg(d, addr);
    uint8_t next = (uint8_t)((cur & ~mask) | (val & mask));
    if (next != cur) {
        st25r3916_write_reg(d, addr, next);
    }
}

uint8_t st25r3916_read_regb(st25r_t *d, uint8_t addr)
{
    uint8_t tx[3] = {ST25R_SPI_SPACEB_PREFIX, ST25R_SPI_REG_READ(addr), 0};
    uint8_t rx[3];
    txn(d, tx, rx, 3);
    return rx[2]; /* data clocked out after the prefix and mode bytes */
}

void st25r3916_write_regb(st25r_t *d, uint8_t addr, uint8_t val)
{
    uint8_t tx[3] = {ST25R_SPI_SPACEB_PREFIX, ST25R_SPI_REG_WRITE(addr), val};
    txn(d, tx, NULL, 3);
}

void st25r3916_write_fifo(st25r_t *d, const uint8_t *data, size_t n)
{
    if (n == 0 || n > ST25R_FIFO_DEPTH) {
        return;
    }
    uint8_t hdr = ST25R_SPI_FIFO_LOAD;
    /* The board's txn does one CS frame; load the FIFO as the mode byte then the data in a
     * single transaction. We avoid a big stack copy by sending the header and the data back
     * to back inside one select, which the board keeps framed. */
    uint8_t buf[1 + ST25R_FIFO_DEPTH];
    buf[0] = hdr;
    memcpy(buf + 1, data, n);
    txn(d, buf, NULL, n + 1);
}

void st25r3916_read_fifo(st25r_t *d, uint8_t *out, size_t n)
{
    if (n == 0 || n > ST25R_FIFO_DEPTH) {
        return;
    }
    uint8_t tx[1 + ST25R_FIFO_DEPTH];
    uint8_t rx[1 + ST25R_FIFO_DEPTH];
    tx[0] = ST25R_SPI_FIFO_READ;
    memset(tx + 1, 0, n);
    txn(d, tx, rx, n + 1);
    memcpy(out, rx + 1, n);
}

int st25r3916_fifo_count(st25r_t *d)
{
    uint8_t s[2];
    st25r3916_read_regs(d, ST25R_REG_FIFO_STATUS1, s, 2);
    int count = s[0] | ((s[1] >> ST25R_FIFO2_B9_B8_SHIFT) << 8);
    return count;
}

uint32_t st25r3916_get_interrupts(st25r_t *d)
{
    uint8_t s[4] = {0};
    st25r3916_read_regs(d, ST25R_REG_MAIN_IRQ, s, 4); /* reading clears them */
    return (uint32_t)s[0] | ((uint32_t)s[1] << 8) | ((uint32_t)s[2] << 16) | ((uint32_t)s[3] << 24);
}

void st25r3916_set_interrupt_mask(st25r_t *d, uint32_t mask)
{
    st25r3916_write_reg(d, ST25R_REG_MASK_MAIN_IRQ, (uint8_t)(mask));
    st25r3916_write_reg(d, ST25R_REG_MASK_TIMER_NFC_IRQ, (uint8_t)(mask >> 8));
    st25r3916_write_reg(d, ST25R_REG_MASK_ERROR_WUP_IRQ, (uint8_t)(mask >> 16));
    st25r3916_write_reg(d, ST25R_REG_MASK_PASSIVE_TARGET_IRQ, (uint8_t)(mask >> 24));
}

bool st25r3916_probe(st25r_t *d)
{
    uint8_t id = st25r3916_read_reg(d, ST25R_REG_IC_IDENTITY);
    d->ic_type = (uint8_t)((id & ST25R_IC_TYPE_MASK) >> ST25R_IC_TYPE_SHIFT);
    d->ic_rev = (uint8_t)(id & ST25R_IC_REV_MASK);
    return d->ic_type == ST25R_IC_TYPE_ST25R3916B;
}
