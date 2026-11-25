/*
 * test_st25r3916.c — the ST25R3916B driver's SPI framing and register access, against a fake
 * bus that records every transaction and returns a small emulated register file. Verifies the
 * byte-level protocol from DS13541 §4.3.3/§4.4 without a chip.
 */
#include <string.h>

#include "st25r3916.h"
#include "st25r3916_regs.h"
#include "test.h"

/* A fake ST25R3916B: a 64-byte space-A register file plus space-B, with the IC identity
 * register pre-loaded, enough to exercise read/write/auto-increment/commands. */
static uint8_t g_rega[64];
static uint8_t g_regb[64];
static uint8_t g_fifo[512];
static int g_fifo_len;
static uint8_t g_last_cmd;
static uint8_t g_last_tx[520];
static size_t g_last_n;

static void fake_txn(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n)
{
    (void)ctx;
    g_last_n = n < sizeof(g_last_tx) ? n : sizeof(g_last_tx);
    if (tx) {
        memcpy(g_last_tx, tx, g_last_n);
    }
    if (rx) {
        memset(rx, 0, n);
    }
    if (!tx || n == 0) {
        return;
    }
    uint8_t op = tx[0];
    if (op == ST25R_SPI_SPACEB_PREFIX) { /* space-B: prefix + mode byte + data */
        uint8_t mode = tx[1];
        uint8_t addr = mode & 0x3F;
        if ((mode & 0xC0) == 0x40) { /* read */
            if (rx) {
                rx[2] = g_regb[addr & 0x3F];
            }
        } else { /* write */
            g_regb[addr & 0x3F] = tx[2];
        }
        return;
    }
    if ((op & 0xC0) == 0x00) { /* register write, auto-increment */
        uint8_t addr = op & 0x3F;
        for (size_t i = 1; i < n; i++) {
            g_rega[(addr + (i - 1)) & 0x3F] = tx[i];
        }
    } else if ((op & 0xC0) == 0x40) { /* register read, auto-increment */
        uint8_t addr = op & 0x3F;
        if (rx) {
            for (size_t i = 1; i < n; i++) {
                rx[i] = g_rega[(addr + (i - 1)) & 0x3F];
            }
        }
    } else if (op == ST25R_SPI_FIFO_LOAD) {
        g_fifo_len = (int)(n - 1);
        memcpy(g_fifo, tx + 1, g_fifo_len);
    } else if (op == ST25R_SPI_FIFO_READ) {
        if (rx) {
            memcpy(rx + 1, g_fifo, n - 1);
        }
    } else if ((op & 0xC0) == 0xC0 && op != ST25R_SPI_SPACEB_PREFIX) { /* direct command */
        g_last_cmd = op;
    }
}

static void fake_delay(void *ctx, unsigned us)
{
    (void)ctx;
    (void)us;
}

int main(void)
{
    memset(g_rega, 0, sizeof(g_rega));
    memset(g_regb, 0, sizeof(g_regb));
    /* IC identity: ST25R3916B (type 00110b) at rev 4.1 (001b) -> 0x31. */
    g_rega[ST25R_REG_IC_IDENTITY] = (uint8_t)((ST25R_IC_TYPE_ST25R3916B << ST25R_IC_TYPE_SHIFT) | 1);

    st25r_bus_t bus = {.ctx = NULL, .txn = fake_txn, .delay_us = fake_delay};
    st25r_t d = {.bus = &bus};

    /* ---- probe: the part identifies as an ST25R3916B ---- */
    CHECK(st25r3916_probe(&d));
    CHECK_EQ(d.ic_type, 0x06);
    CHECK_EQ(d.ic_rev, 1);
    /* The read framed one mode byte 0x40|0x3F then one data byte. */
    CHECK_EQ(g_last_tx[0], 0x7F);
    CHECK_EQ(g_last_n, 2);

    /* A non-B part is rejected (ic_type 0x05). */
    g_rega[ST25R_REG_IC_IDENTITY] = (uint8_t)(0x05 << ST25R_IC_TYPE_SHIFT);
    CHECK(!st25r3916_probe(&d));

    /* ---- register write/read, and the mode-byte encoding ---- */
    st25r3916_write_reg(&d, ST25R_REG_OP_CONTROL, ST25R_OP_EN | ST25R_OP_RX_EN);
    CHECK_EQ(g_last_tx[0], 0x02);             /* 00 + address 0x02 */
    CHECK_EQ(g_last_tx[1], 0xC0);             /* en | rx_en */
    CHECK_EQ(g_rega[ST25R_REG_OP_CONTROL], 0xC0);
    CHECK_EQ(st25r3916_read_reg(&d, ST25R_REG_OP_CONTROL), 0xC0);
    CHECK_EQ(g_last_tx[0], 0x42);             /* 01 + address 0x02 */

    /* modify_reg is read-modify-write. */
    st25r3916_modify_reg(&d, ST25R_REG_OP_CONTROL, ST25R_OP_TX_EN, ST25R_OP_TX_EN);
    CHECK_EQ(g_rega[ST25R_REG_OP_CONTROL], 0xC8);

    /* ---- auto-increment burst read of the four interrupt registers ---- */
    g_rega[ST25R_REG_MAIN_IRQ] = ST25R_IRQ_RXE | ST25R_IRQ_TXE;
    g_rega[ST25R_REG_TIMER_NFC_IRQ] = ST25R_IRQ_DCT;
    g_rega[ST25R_REG_ERROR_WUP_IRQ] = ST25R_IRQ_CRC;
    g_rega[ST25R_REG_PASSIVE_TARGET_IRQ] = 0x00;
    uint32_t irq = st25r3916_get_interrupts(&d);
    CHECK_EQ(g_last_tx[0], 0x5A); /* 01 + 0x1A */
    CHECK_EQ(g_last_n, 5);        /* mode byte + 4 */
    CHECK_EQ(ST25R_IRQ_MAIN(irq), ST25R_IRQ_RXE | ST25R_IRQ_TXE);
    CHECK_EQ(ST25R_IRQ_TIMER_NFC(irq), ST25R_IRQ_DCT);
    CHECK_EQ(ST25R_IRQ_ERROR_WUP(irq), ST25R_IRQ_CRC);

    /* interrupt mask: four writes to 0x16..0x19 from the packed value. */
    st25r3916_set_interrupt_mask(&d, 0x00FF0000u); /* mask all error/wake-up IRQs */
    CHECK_EQ(g_rega[ST25R_REG_MASK_ERROR_WUP_IRQ], 0xFF);
    CHECK_EQ(g_rega[ST25R_REG_MASK_MAIN_IRQ], 0x00);

    /* ---- direct command ---- */
    st25r3916_cmd(&d, ST25R_CMD_SET_DEFAULT);
    CHECK_EQ(g_last_cmd, 0xC1);
    CHECK_EQ(g_last_n, 1);

    /* ---- FIFO load/read round-trip and the byte-count status ---- */
    uint8_t payload[] = {0x26, 0x01, 0x02, 0x03};
    st25r3916_write_fifo(&d, payload, sizeof(payload));
    CHECK_EQ(g_last_tx[0], ST25R_SPI_FIFO_LOAD);
    CHECK_EQ(g_fifo_len, (int)sizeof(payload));
    uint8_t back[4] = {0};
    st25r3916_read_fifo(&d, back, sizeof(back));
    CHECK_EQ(g_last_tx[0], ST25R_SPI_FIFO_READ);
    CHECK(!memcmp(back, payload, sizeof(payload)));

    /* fifo_count decodes the 10-bit count across FIFO status 1/2 (e.g. 300 bytes). */
    g_rega[ST25R_REG_FIFO_STATUS1] = 300 & 0xFF;
    g_rega[ST25R_REG_FIFO_STATUS2] = (uint8_t)((300 >> 8) << ST25R_FIFO2_B9_B8_SHIFT);
    CHECK_EQ(st25r3916_fifo_count(&d), 300);

    /* ---- space-B register access uses the 0xFB prefix ---- */
    st25r3916_write_regb(&d, ST25R_REGB_EMD_SUPPRESSION, 0x5A);
    CHECK_EQ(g_last_tx[0], ST25R_SPI_SPACEB_PREFIX);
    CHECK_EQ(g_last_tx[1], 0x05); /* 00 + addr 0x05 */
    CHECK_EQ(st25r3916_read_regb(&d, ST25R_REGB_EMD_SUPPRESSION), 0x5A);
    CHECK_EQ(g_last_tx[1], 0x45); /* 01 + addr 0x05 */

    TEST_MAIN_END();
}
