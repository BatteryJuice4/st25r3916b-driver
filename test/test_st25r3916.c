/*
 * test_st25r3916.c — the ST25R3916B driver against a fake bus + a fake NFC-A card.
 *
 * The fake models enough of the chip to exercise the real driver end to end: the SPI framing
 * (DS13541 §4.3.3), the interrupt registers (set by "the chip" in response to commands and
 * cleared on read), the FIFO, the osc_ok status, and a single ISO 14443-A card that answers
 * REQA and the anticollision/SELECT cascade (DS13541 §4.2.13). No real hardware.
 */
#include <string.h>

#include "st25r3916.h"
#include "st25r3916_regs.h"
#include "test.h"

/* ---- Fake chip + card state ------------------------------------------------------------ */
static uint8_t g_rega[64];
static uint8_t g_regb[64];
static uint8_t g_fifo[512]; /* current FIFO contents (what a read returns) */
static int g_fifo_len;
static uint8_t g_txbuf[512]; /* the last frame the host loaded, for the card to interpret */
static int g_txlen;
static uint32_t g_irq;      /* pending interrupts, packed like st25r3916_get_interrupts() */
static uint32_t g_deferred; /* interrupts delivered on the *next* interrupt read (TXE then RXE) */
static uint8_t g_last_tx[520];
static size_t g_last_n;
static uint8_t g_last_cmd;

/* The emulated card. */
static uint8_t g_uid[10];
static int g_uidlen;
static uint8_t g_atqa[2];
static uint8_t g_sak_final;
static bool g_card_present;

#define FI_M(bit) ((uint32_t)(bit))
#define FI_T(bit) ((uint32_t)(bit) << 8)

/* Build cascade level `lvl`'s 4 anticollision bytes (CLn) for the card's UID. */
static int card_cln(int lvl, uint8_t out[4])
{
    if (g_uidlen == 4) {
        if (lvl != 0) {
            return 0;
        }
        memcpy(out, g_uid, 4);
        return 1;
    }
    if (g_uidlen == 7) {
        if (lvl == 0) {
            out[0] = 0x88; /* cascade tag */
            out[1] = g_uid[0];
            out[2] = g_uid[1];
            out[3] = g_uid[2];
            return 1;
        }
        if (lvl == 1) {
            memcpy(out, g_uid + 3, 4);
            return 1;
        }
    }
    return 0;
}

static bool card_level_is_final(int lvl)
{
    return (g_uidlen == 4) || (g_uidlen == 7 && lvl == 1);
}

static void card_transmit(void)
{
    /* The host loaded a SEL frame into g_txbuf: [SEL, NVB, ...]. */
    static const uint8_t SEL[3] = {0x93, 0x95, 0x97};
    int lvl = -1;
    for (int i = 0; i < 3; i++) {
        if (g_txlen >= 1 && g_txbuf[0] == SEL[i]) {
            lvl = i;
        }
    }
    g_fifo_len = 0;
    if (g_card_present && lvl >= 0 && g_txlen >= 2) {
        uint8_t nvb = g_txbuf[1];
        uint8_t cln[4];
        if (!card_cln(lvl, cln)) {
            /* no such level for this card: no answer */
        } else if (nvb == 0x20) { /* ANTICOLLISION: UID bytes + BCC */
            memcpy(g_fifo, cln, 4);
            g_fifo[4] = (uint8_t)(cln[0] ^ cln[1] ^ cln[2] ^ cln[3]);
            g_fifo_len = 5;
        } else if (nvb == 0x70) { /* SELECT: SAK (+ 2 CRC bytes) */
            g_fifo[0] = card_level_is_final(lvl) ? g_sak_final : 0x04; /* 0x04 = cascade bit */
            g_fifo[1] = 0x00;
            g_fifo[2] = 0x00; /* fake CRC; the chip checks it, the driver does not */
            g_fifo_len = 3;
        }
    }
    /* Transmit finishes first (I_txe); the reply (I_rxe) or the no-response timeout (I_nre)
     * arrives as a separate, later interrupt. */
    g_irq |= FI_M(ST25R_IRQ_TXE);
    g_deferred |= (g_fifo_len > 0) ? FI_M(ST25R_IRQ_RXE) : FI_T(ST25R_IRQ_NRE);
}

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

    if (op == ST25R_SPI_SPACEB_PREFIX) {
        uint8_t mode = tx[1], addr = mode & 0x3F;
        if ((mode & 0xC0) == 0x40) {
            if (rx) {
                rx[2] = g_regb[addr];
            }
        } else {
            g_regb[addr] = tx[2];
        }
        return;
    }
    if ((op & 0xC0) == 0x00) { /* register write (auto-increment) */
        uint8_t addr = op & 0x3F;
        for (size_t i = 1; i < n; i++) {
            g_rega[(addr + (i - 1)) & 0x3F] = tx[i];
        }
        return;
    }
    if ((op & 0xC0) == 0x40) { /* register read (auto-increment), with special registers */
        uint8_t addr = op & 0x3F;
        if (!rx) {
            return;
        }
        bool touched_irq = false;
        for (size_t i = 1; i < n; i++) {
            uint8_t a = (uint8_t)((addr + (i - 1)) & 0x3F);
            uint8_t v;
            if (a >= ST25R_REG_MAIN_IRQ && a <= ST25R_REG_PASSIVE_TARGET_IRQ) {
                int shift = (a - ST25R_REG_MAIN_IRQ) * 8;
                v = (uint8_t)(g_irq >> shift);
                g_irq &= ~((uint32_t)0xFF << shift); /* reading clears */
                touched_irq = true;
            } else if (a == ST25R_REG_FIFO_STATUS1) {
                v = (uint8_t)(g_fifo_len & 0xFF);
            } else if (a == ST25R_REG_FIFO_STATUS2) {
                v = (uint8_t)(((g_fifo_len >> 8) & 0x3) << ST25R_FIFO2_B9_B8_SHIFT);
            } else if (a == ST25R_REG_AUX_DISPLAY) {
                v = (g_rega[ST25R_REG_OP_CONTROL] & ST25R_OP_EN) ? ST25R_AUX_OSC_OK : 0;
            } else {
                v = g_rega[a];
            }
            rx[i] = v;
        }
        if (touched_irq) {
            g_irq |= g_deferred; /* the next event (RXE/NRE) becomes visible after this read */
            g_deferred = 0;
        }
        return;
    }
    if (op == ST25R_SPI_FIFO_LOAD) {
        g_fifo_len = (int)(n - 1);
        memcpy(g_fifo, tx + 1, g_fifo_len);
        g_txlen = g_fifo_len;
        memcpy(g_txbuf, tx + 1, g_txlen);
        return;
    }
    if (op == ST25R_SPI_FIFO_READ) {
        if (rx) {
            memcpy(rx + 1, g_fifo, n - 1);
        }
        return;
    }
    if ((op & 0xC0) == 0xC0) { /* direct command */
        g_last_cmd = op;
        switch (op) {
        case ST25R_CMD_SET_DEFAULT:
            memset(g_rega, 0, sizeof(g_rega));
            g_irq = 0;
            g_deferred = 0;
            g_fifo_len = 0;
            break;
        case ST25R_CMD_STOP:
        case ST25R_CMD_CLEAR_FIFO:
            g_fifo_len = 0;
            break;
        case ST25R_CMD_ADJUST_REGULATORS:
            g_irq |= FI_T(ST25R_IRQ_DCT);
            break;
        case ST25R_CMD_INITIAL_FIELD_ON:
            g_irq |= FI_T(ST25R_IRQ_CAT);
            break;
        case ST25R_CMD_TRANSMIT_REQA:
        case ST25R_CMD_TRANSMIT_WUPA:
            g_irq |= FI_M(ST25R_IRQ_TXE);
            if (g_card_present) {
                memcpy(g_fifo, g_atqa, 2);
                g_fifo_len = 2;
                g_deferred |= FI_M(ST25R_IRQ_RXE);
            } else {
                g_fifo_len = 0;
                g_deferred |= FI_T(ST25R_IRQ_NRE);
            }
            break;
        case ST25R_CMD_TRANSMIT_WITH_CRC:
        case ST25R_CMD_TRANSMIT_WITHOUT_CRC:
            card_transmit();
            break;
        default:
            break;
        }
    }
}

static void fake_delay(void *ctx, unsigned us)
{
    (void)ctx;
    (void)us;
}
static bool fake_wait_irq(void *ctx, int timeout_ms)
{
    (void)ctx;
    (void)timeout_ms;
    return g_irq != 0; /* the "pin" is active whenever an interrupt is pending */
}

static st25r_bus_t g_bus = {
    .ctx = NULL, .txn = fake_txn, .delay_us = fake_delay, .wait_irq = fake_wait_irq};

static void reset_chip(void)
{
    memset(g_rega, 0, sizeof(g_rega));
    memset(g_regb, 0, sizeof(g_regb));
    g_irq = 0;
    g_deferred = 0;
    g_fifo_len = 0;
    g_txlen = 0;
    g_rega[ST25R_REG_IC_IDENTITY] =
        (uint8_t)((ST25R_IC_TYPE_ST25R3916B << ST25R_IC_TYPE_SHIFT) | 1); /* 3916B rev 4.1 */
}

int main(void)
{
    reset_chip();
    st25r_t d = {.bus = &g_bus};

    /* ---- probe ---- */
    CHECK(st25r3916_probe(&d));
    CHECK_EQ(d.ic_type, 0x06);
    CHECK_EQ(d.ic_rev, 1);
    CHECK_EQ(g_last_tx[0], 0x7F); /* read 01 + 0x3F */
    CHECK_EQ(g_last_n, 2);
    g_rega[ST25R_REG_IC_IDENTITY] = (uint8_t)(0x05 << ST25R_IC_TYPE_SHIFT); /* non-B part */
    CHECK(!st25r3916_probe(&d));
    reset_chip();

    /* ---- SPI framing: register and command bytes ---- */
    st25r3916_write_reg(&d, ST25R_REG_OP_CONTROL, ST25R_OP_EN | ST25R_OP_RX_EN);
    CHECK_EQ(g_last_tx[0], 0x02); /* 00 + addr 0x02 */
    CHECK_EQ(g_last_tx[1], 0xC0);
    CHECK_EQ(st25r3916_read_reg(&d, ST25R_REG_OP_CONTROL), 0xC0);
    CHECK_EQ(g_last_tx[0], 0x42); /* 01 + addr 0x02 */
    st25r3916_modify_reg(&d, ST25R_REG_OP_CONTROL, ST25R_OP_TX_EN, ST25R_OP_TX_EN);
    CHECK_EQ(g_rega[ST25R_REG_OP_CONTROL], 0xC8);
    st25r3916_write_regb(&d, ST25R_REGB_EMD_SUPPRESSION, 0x5A);
    CHECK_EQ(g_last_tx[0], ST25R_SPI_SPACEB_PREFIX);
    CHECK_EQ(g_last_tx[1], 0x05);
    CHECK_EQ(st25r3916_read_regb(&d, ST25R_REGB_EMD_SUPPRESSION), 0x5A);
    st25r3916_cmd(&d, ST25R_CMD_CLEAR_RSSI);
    CHECK_EQ(g_last_cmd, 0xDA);

    /* ---- interrupt read is a 4-register burst, packed little-endian, cleared on read ---- */
    g_irq = FI_M(ST25R_IRQ_RXE | ST25R_IRQ_TXE) | FI_T(ST25R_IRQ_DCT);
    uint32_t irq = st25r3916_get_interrupts(&d);
    CHECK_EQ(g_last_tx[0], 0x5A); /* read 01 + 0x1A */
    CHECK_EQ(g_last_n, 5);
    CHECK_EQ(ST25R_IRQ_MAIN(irq), ST25R_IRQ_RXE | ST25R_IRQ_TXE);
    CHECK_EQ(ST25R_IRQ_TIMER_NFC(irq), ST25R_IRQ_DCT);
    CHECK_EQ(st25r3916_get_interrupts(&d), 0u);

    /* ---- interrupt mask: four writes to 0x16..0x19 from the packed value ---- */
    st25r3916_set_interrupt_mask(&d, 0x00FF0000u); /* mask all error/wake-up IRQs */
    CHECK_EQ(g_rega[ST25R_REG_MASK_ERROR_WUP_IRQ], 0xFF);
    CHECK_EQ(g_rega[ST25R_REG_MASK_MAIN_IRQ], 0x00);

    /* ---- FIFO round-trip and the byte count ---- */
    uint8_t payload[] = {0x26, 0x01, 0x02, 0x03};
    st25r3916_write_fifo(&d, payload, sizeof(payload));
    CHECK_EQ(g_last_tx[0], ST25R_SPI_FIFO_LOAD);
    uint8_t back[4] = {0};
    st25r3916_read_fifo(&d, back, sizeof(back));
    CHECK_EQ(g_last_tx[0], ST25R_SPI_FIFO_READ);
    CHECK(!memcmp(back, payload, sizeof(payload)));
    CHECK_EQ(st25r3916_fifo_count(&d), (int)sizeof(payload));

    /* ================= RF flows ================= */

    /* ---- power-up: SET_DEFAULT, osc_ok, adjust regulators, field on ---- */
    reset_chip();
    CHECK_EQ(st25r3916_field_on(&d), ST25R_OK);
    CHECK(g_rega[ST25R_REG_OP_CONTROL] & ST25R_OP_EN);    /* Ready */
    CHECK(g_rega[ST25R_REG_OP_CONTROL] & ST25R_OP_TX_EN); /* transmitter on */
    CHECK_EQ(g_rega[ST25R_REG_MODE], ST25R_OM_ISO14443A << ST25R_MODE_OM_SHIFT);

    /* ---- a 4-byte-UID card (e.g. MIFARE Classic 1K): REQA + one cascade level ---- */
    g_card_present = true;
    g_uidlen = 4;
    memcpy(g_uid, (uint8_t[]){0x04, 0x11, 0x22, 0x33}, 4);
    g_atqa[0] = 0x04;
    g_atqa[1] = 0x00;
    g_sak_final = 0x08;
    st25r_nfca_t card;
    CHECK_EQ(st25r3916_nfca_select(&d, &card), ST25R_OK);
    CHECK_EQ(card.uid_len, 4);
    CHECK(!memcmp(card.uid, g_uid, 4));
    CHECK_EQ(card.sak, 0x08);
    CHECK_EQ(card.atqa[0], 0x04);

    /* ---- a 7-byte-UID card (e.g. NTAG): two cascade levels, cascade-tag handling ---- */
    g_uidlen = 7;
    memcpy(g_uid, (uint8_t[]){0x04, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}, 7);
    g_atqa[0] = 0x44;
    g_atqa[1] = 0x00;
    g_sak_final = 0x00;
    CHECK_EQ(st25r3916_nfca_select(&d, &card), ST25R_OK);
    CHECK_EQ(card.uid_len, 7);
    CHECK(!memcmp(card.uid, g_uid, 7)); /* cascade tag stripped, bytes reassembled */
    CHECK_EQ(card.sak, 0x00);

    /* ---- no card in the field: REQA times out ---- */
    g_card_present = false;
    CHECK_EQ(st25r3916_nfca_select(&d, &card), ST25R_ERR_TIMEOUT);

    /* ---- transceive reports a buffer-too-small error cleanly ---- */
    g_card_present = true;
    g_uidlen = 4;
    memcpy(g_uid, (uint8_t[]){0x01, 0x02, 0x03, 0x04}, 4);
    uint8_t sel[2] = {0x93, 0x20};
    uint8_t tiny[2];
    CHECK_EQ(st25r3916_transceive(&d, sel, 16, ST25R_TX_ANTICOL, tiny, sizeof(tiny), NULL, 20),
             ST25R_ERR_NOMEM); /* the 5-byte answer does not fit 2 bytes */

    TEST_MAIN_END();
}
