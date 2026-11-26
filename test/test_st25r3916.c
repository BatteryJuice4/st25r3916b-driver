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

/* The emulated cards. Card 0 is the one most tests use (the g_uid.. names below); others
 * join it in the field for the multi-card anticollision tests. States per ISO 14443-3. */
enum { CARD_IDLE = 0, CARD_READY, CARD_ACTIVE, CARD_HALT };
typedef struct {
    bool present;
    uint8_t uid[10];
    int uidlen;
    uint8_t atqa[2];
    uint8_t sak_final;
    int state;
    int lvl; /* cascade level the card is at while READY */
} fake_card_t;
#define NCARDS 3
static fake_card_t g_cards[NCARDS];
#define g_uid (g_cards[0].uid)
#define g_uidlen (g_cards[0].uidlen)
#define g_atqa (g_cards[0].atqa)
#define g_sak_final (g_cards[0].sak_final)
#define g_card_present (g_cards[0].present)

static bool g_bad_coll; /* report a nonsense collision position */

static void add_card(int k, const uint8_t *uid, int len, uint8_t sak)
{
    g_cards[k].present = true;
    memcpy(g_cards[k].uid, uid, len);
    g_cards[k].uidlen = len;
    g_cards[k].atqa[0] = len == 4 ? 0x04 : 0x44;
    g_cards[k].sak_final = sak;
    g_cards[k].state = CARD_IDLE;
}

/* Listen (card-emulation) mode: PT_Memory the host loaded into us. */
static uint8_t g_pt[16];
static int g_pt_len;

/* Wake-up mode: the antenna amplitude/phase a measurement command would report. */
static uint8_t g_sim_amp = 0x5C;
static uint8_t g_sim_phase = 0x33;

#define FI_M(bit) ((uint32_t)(bit))
#define FI_T(bit) ((uint32_t)(bit) << 8)

/* Build cascade level `lvl`'s 5 anticollision bytes (CLn + BCC) for a card's UID. */
static int card_cln(const fake_card_t *c, int lvl, uint8_t out[5])
{
    if (c->uidlen == 4 && lvl == 0) {
        memcpy(out, c->uid, 4);
    } else if (c->uidlen == 7 && lvl == 0) {
        out[0] = 0x88; /* cascade tag */
        memcpy(out + 1, c->uid, 3);
    } else if (c->uidlen == 7 && lvl == 1) {
        memcpy(out, c->uid + 3, 4);
    } else {
        return 0;
    }
    out[4] = (uint8_t)(out[0] ^ out[1] ^ out[2] ^ out[3]);
    return 1;
}

static bool card_level_is_final(const fake_card_t *c, int lvl)
{
    return (c->uidlen == 4) || (c->uidlen == 7 && lvl == 1);
}

static int bit_of(const uint8_t *b, int i)
{
    return (b[i / 8] >> (i % 8)) & 1;
}

static void card_transmit(void)
{
    /* The host loaded a frame into g_txbuf: [SEL, NVB, ...] or HLTA [50 00]. */
    static const uint8_t SEL[3] = {0x93, 0x95, 0x97};
    int lvl = -1;
    for (int i = 0; i < 3; i++) {
        if (g_txlen >= 1 && g_txbuf[0] == SEL[i]) {
            lvl = i;
        }
    }
    g_fifo_len = 0;
    bool collision = false;

    if (g_txlen >= 2 && g_txbuf[0] == 0x50 && g_txbuf[1] == 0x00) { /* HLTA: no answer */
        for (int k = 0; k < NCARDS; k++) {
            if (g_cards[k].present && g_cards[k].state == CARD_ACTIVE) {
                g_cards[k].state = CARD_HALT;
            }
        }
    } else if (lvl >= 0 && g_txlen >= 2 && g_txbuf[1] == 0x70) { /* SELECT: SAK + CRC */
        for (int k = 0; k < NCARDS; k++) {
            fake_card_t *c = &g_cards[k];
            uint8_t cln[5];
            if (!c->present || c->state != CARD_READY || c->lvl != lvl || !card_cln(c, lvl, cln)) {
                continue;
            }
            if (g_txlen >= 7 && !memcmp(cln, g_txbuf + 2, 5)) {
                bool fin = card_level_is_final(c, lvl);
                g_fifo[0] = fin ? c->sak_final : 0x04; /* 0x04 = cascade bit */
                g_fifo[1] = 0x00;
                g_fifo[2] = 0x00; /* fake CRC; the chip checks it, the driver does not */
                g_fifo_len = 3;
                if (fin) {
                    c->state = CARD_ACTIVE;
                } else {
                    c->lvl++;
                }
            } else {
                c->state = CARD_IDLE; /* another card was selected */
            }
        }
    } else if (lvl >= 0 && g_txlen >= 2) { /* ANTICOLLISION with `known` UID bits */
        uint8_t nvb = g_txbuf[1];
        int nbytes = nvb >> 4, nbits = nvb & 7;
        int known = (nbytes - 2) * 8 + nbits;
        uint8_t img[7] = {0}; /* the frame image as the reader hears it (answers ORed) */
        uint8_t first[5];
        int n = 0, coll = 40;
        for (int k = 0; k < NCARDS; k++) {
            fake_card_t *c = &g_cards[k];
            uint8_t cln[5];
            if (!c->present || c->state != CARD_READY || c->lvl != lvl || !card_cln(c, lvl, cln)) {
                continue;
            }
            bool match = known >= 0 && known < 40;
            for (int i = 0; match && i < known; i++) {
                match = bit_of(cln, i) == bit_of(g_txbuf + 2, i);
            }
            if (!match) {
                continue;
            }
            if (n++ == 0) {
                memcpy(first, cln, 5);
            } else {
                for (int i = known; i < 40; i++) {
                    if (bit_of(cln, i) != bit_of(first, i)) {
                        if (i < coll) {
                            coll = i;
                        }
                        break;
                    }
                }
            }
            for (int i = 0; i < 5; i++) {
                img[2 + i] |= cln[i];
            }
        }
        if (n > 0) {
            /* The answer starts at the split position, aligned within the first byte. */
            g_fifo_len = 7 - nbytes;
            memcpy(g_fifo, img + nbytes, g_fifo_len);
            g_fifo[0] &= (uint8_t)~((1u << nbits) - 1u);
            if (n > 1 && coll < 40) {
                collision = true;
                int pos = g_bad_coll ? 0 : 16 + coll;
                g_rega[ST25R_REG_COLLISION] =
                    (uint8_t)(((pos / 8) << ST25R_COLL_BYTE_SHIFT) | ((pos % 8) << ST25R_COLL_BIT_SHIFT));
            }
        }
    }
    /* Transmit finishes first (I_txe); the reply (I_rxe) or the no-response timeout (I_nre)
     * arrives as a separate, later interrupt. */
    g_irq |= FI_M(ST25R_IRQ_TXE);
    g_deferred |= (g_fifo_len > 0) ? FI_M(ST25R_IRQ_RXE) : FI_T(ST25R_IRQ_NRE);
    if (collision) {
        g_deferred |= FI_M(ST25R_IRQ_COL);
    }
}

/* REQA wakes IDLE cards (this fake is lenient: any card not halted), WUPA wakes halted ones
 * too. All woken cards answer at once; their ATQAs OR together on the air. */
static int cards_request(bool wupa)
{
    int n = 0;
    g_fifo[0] = g_fifo[1] = 0;
    for (int k = 0; k < NCARDS; k++) {
        fake_card_t *c = &g_cards[k];
        if (!c->present || (c->state == CARD_HALT && !wupa)) {
            continue;
        }
        c->state = CARD_READY;
        c->lvl = 0;
        g_fifo[0] |= c->atqa[0];
        g_fifo[1] |= c->atqa[1];
        n++;
    }
    return n;
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
    if (op == ST25R_SPI_PT_LOAD_A) { /* load PT_Memory (card-emulation identity) */
        g_pt_len = (int)(n - 1);
        if (g_pt_len > (int)sizeof(g_pt)) {
            g_pt_len = sizeof(g_pt);
        }
        memcpy(g_pt, tx + 1, g_pt_len);
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
            if (cards_request(op == ST25R_CMD_TRANSMIT_WUPA) > 0) {
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
        case ST25R_CMD_MEASURE_AMPLITUDE:
            g_rega[ST25R_REG_AMPLITUDE_MEAS_DISPLAY] = g_sim_amp;
            g_irq |= FI_T(ST25R_IRQ_DCT);
            break;
        case ST25R_CMD_MEASURE_PHASE:
            g_rega[ST25R_REG_PHASE_MEAS_DISPLAY] = g_sim_phase;
            g_irq |= FI_T(ST25R_IRQ_DCT);
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
    uint8_t atqa[2];
    CHECK_EQ(st25r3916_nfca_request(&d, atqa, false), ST25R_OK); /* card to READY */
    uint8_t sel[2] = {0x93, 0x20};
    uint8_t tiny[2];
    CHECK_EQ(st25r3916_transceive(&d, sel, 16, ST25R_TX_ANTICOL, tiny, sizeof(tiny), NULL, 20),
             ST25R_ERR_NOMEM); /* the 5-byte answer does not fit 2 bytes */

    /* ================= Several cards: anticollision resolution ================= */

    /* ---- two 4-byte cards differing in one bit: the 1-branch wins, HLTA finds the other ---- */
    reset_chip();
    memset(g_cards, 0, sizeof(g_cards));
    add_card(0, (uint8_t[]){0x04, 0x11, 0x22, 0x33}, 4, 0x08);
    add_card(1, (uint8_t[]){0x04, 0x11, 0x2A, 0x33}, 4, 0x08); /* byte 2, bit 3 differs */
    CHECK_EQ(st25r3916_nfca_select(&d, &card), ST25R_OK);
    CHECK(card.collided);
    CHECK_EQ(card.uid_len, 4);
    CHECK(!memcmp(card.uid, g_cards[1].uid, 4)); /* the collided bit was chosen as 1 */
    CHECK_EQ(g_cards[1].state, CARD_ACTIVE);
    CHECK_EQ(g_cards[0].state, CARD_IDLE); /* dropped out at the SELECT */
    CHECK_EQ(st25r3916_nfca_halt(&d), ST25R_OK);
    CHECK_EQ(g_cards[1].state, CARD_HALT);
    CHECK_EQ(st25r3916_nfca_select(&d, &card), ST25R_OK);
    CHECK(!card.collided);
    CHECK(!memcmp(card.uid, g_cards[0].uid, 4));
    CHECK_EQ(st25r3916_nfca_halt(&d), ST25R_OK);
    CHECK_EQ(st25r3916_nfca_select(&d, &card), ST25R_ERR_TIMEOUT); /* all halted */
    uint8_t wq[2];
    CHECK_EQ(st25r3916_nfca_request(&d, wq, true), ST25R_OK); /* WUPA wakes halted cards */

    /* ---- collisions on the very first and the very last UID bit ---- */
    reset_chip();
    memset(g_cards, 0, sizeof(g_cards));
    add_card(0, (uint8_t[]){0x10, 0x20, 0x30, 0x40}, 4, 0x08);
    add_card(1, (uint8_t[]){0x11, 0x20, 0x30, 0x40}, 4, 0x08); /* bit 0 */
    add_card(2, (uint8_t[]){0x11, 0x20, 0x30, 0xC0}, 4, 0x08); /* and bit 31 */
    int seen = 0;
    for (int i = 0; i < 4; i++) {
        int rc = st25r3916_nfca_select(&d, &card);
        if (rc == ST25R_ERR_TIMEOUT) {
            break;
        }
        CHECK_EQ(rc, ST25R_OK);
        for (int k = 0; k < 3; k++) {
            if (!memcmp(card.uid, g_cards[k].uid, 4)) {
                seen |= 1 << k;
            }
        }
        CHECK_EQ(st25r3916_nfca_halt(&d), ST25R_OK);
    }
    CHECK_EQ(seen, 7);

    /* ---- two 7-byte cards sharing cascade level 1 plus a 4-byte card: all three found ---- */
    reset_chip();
    memset(g_cards, 0, sizeof(g_cards));
    add_card(0, (uint8_t[]){0x04, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}, 7, 0x00);
    add_card(1, (uint8_t[]){0x04, 0xAA, 0xBB, 0x01, 0xDD, 0xEE, 0xFF}, 7, 0x00); /* level 2 */
    add_card(2, (uint8_t[]){0x80, 0x01, 0x02, 0x03}, 4, 0x20);
    seen = 0;
    for (int i = 0; i < 5; i++) {
        int rc = st25r3916_nfca_select(&d, &card);
        if (rc == ST25R_ERR_TIMEOUT) {
            break;
        }
        CHECK_EQ(rc, ST25R_OK);
        for (int k = 0; k < 3; k++) {
            if (card.uid_len == g_cards[k].uidlen && !memcmp(card.uid, g_cards[k].uid, card.uid_len)) {
                seen |= 1 << k;
                CHECK_EQ(card.sak, g_cards[k].sak_final);
            }
        }
        CHECK_EQ(st25r3916_nfca_halt(&d), ST25R_OK);
    }
    CHECK_EQ(seen, 7);

    /* ---- an impossible collision position (before the known bits) is a framing error ---- */
    reset_chip();
    memset(g_cards, 0, sizeof(g_cards));
    add_card(0, (uint8_t[]){0x04, 0x11, 0x22, 0x33}, 4, 0x08);
    add_card(1, (uint8_t[]){0x04, 0x11, 0x2A, 0x33}, 4, 0x08);
    g_bad_coll = true; /* the fake reports byte 0 bit 0 (inside SEL) */
    CHECK_EQ(st25r3916_nfca_select(&d, &card), ST25R_ERR_FRAMING);
    g_bad_coll = false;
    memset(g_cards, 0, sizeof(g_cards));

    /* ================= Listen mode (card emulation) ================= */
    reset_chip();
    g_card_present = false;
    g_pt_len = 0;

    /* ---- start as a 4-byte-UID NFC-A tag: PT_Memory and mode registers are set ---- */
    st25r_nfca_target_t tgt = {
        .nfcid1 = {0x08, 0xDE, 0xAD, 0xBE}, .nfcid1_len = 4, .atqa = {0x44, 0x00}, .sak = 0x20};
    CHECK_EQ(st25r3916_listen_start(&d, &tgt), ST25R_OK);
    CHECK(!memcmp(&g_pt[ST25R_PT_OFF_NFCID1], tgt.nfcid1, 4));
    CHECK_EQ(g_pt[ST25R_PT_OFF_SENS_RES], 0x44);     /* ATQA */
    CHECK_EQ(g_pt[ST25R_PT_OFF_SELR_L1], 0x20);      /* SAK, single cascade level */
    CHECK_EQ(g_rega[ST25R_REG_MODE], ST25R_MODE_TARG | (ST25R_OM_ISO14443A << ST25R_MODE_OM_SHIFT));
    CHECK_EQ(g_rega[ST25R_REG_PASSIVE_TARGET], 2u << ST25R_PT_FDEL_SHIFT);
    CHECK((g_rega[ST25R_REG_OP_CONTROL] & ST25R_OP_RX_EN));
    CHECK_EQ(g_rega[ST25R_REG_OP_CONTROL] & ST25R_OP_EN_FD_MASK, ST25R_OP_EN_FD_AUTO);
    CHECK_EQ(g_last_cmd, ST25R_CMD_GOTO_SENSE);

    /* ---- a reader selects us: the chip raises I_wu_a -> ACTIVATED ---- */
    g_irq |= (uint32_t)ST25R_IRQ_WU_A << 24;
    int rlen = -1;
    uint8_t rbuf[32];
    CHECK_EQ(st25r3916_listen_poll(&d, rbuf, sizeof(rbuf), &rlen, 20), ST25R_LISTEN_ACTIVATED);

    /* ---- the reader sends a frame (e.g. RATS): I_rxe + the frame in the FIFO -> RECEIVED ---- */
    uint8_t rats[] = {0xE0, 0x80};
    memcpy(g_fifo, rats, sizeof(rats));
    g_fifo_len = sizeof(rats);
    g_irq |= FI_M(ST25R_IRQ_RXE);
    CHECK_EQ(st25r3916_listen_poll(&d, rbuf, sizeof(rbuf), &rlen, 20), ST25R_LISTEN_RECEIVED);
    CHECK_EQ(rlen, (int)sizeof(rats));
    CHECK(!memcmp(rbuf, rats, sizeof(rats)));

    /* ---- we answer (e.g. ATS): the bytes reach the FIFO and a transmit is issued ---- */
    uint8_t ats[] = {0x05, 0x78, 0x80, 0x70, 0x02};
    CHECK_EQ(st25r3916_listen_respond(&d, ats, sizeof(ats), true, 20), ST25R_OK);
    CHECK_EQ(g_txlen, (int)sizeof(ats));
    CHECK(!memcmp(g_txbuf, ats, sizeof(ats)));

    /* ---- 7-byte-UID tag: level-1 SAK carries the cascade bit, level 2 is final ---- */
    st25r_nfca_target_t tgt7 = {.nfcid1 = {0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66},
                                .nfcid1_len = 7,
                                .atqa = {0x44, 0x00},
                                .sak = 0x00};
    CHECK_EQ(st25r3916_listen_start(&d, &tgt7), ST25R_OK);
    CHECK(!memcmp(&g_pt[ST25R_PT_OFF_NFCID1], tgt7.nfcid1, 7));
    CHECK_EQ(g_pt[ST25R_PT_OFF_SELR_L1], 0x04); /* cascade bit set */
    CHECK_EQ(g_pt[ST25R_PT_OFF_SELR_L2], 0x00); /* final SAK */

    /* ---- a bad NFCID1 length is rejected ---- */
    st25r_nfca_target_t bad = {.nfcid1_len = 5};
    CHECK_EQ(st25r3916_listen_start(&d, &bad), ST25R_ERR_PARAM);

    /* ================= Wake-up mode (low-power tag detection) ================= */
    reset_chip();
    g_sim_amp = 0x5C;

    /* ---- start: amplitude sensing, fixed reference calibrated from a measurement ---- */
    st25r_wakeup_cfg_t wcfg = {.measure_amplitude = true,
                               .period = 2, /* (2+1) x 100 ms = 300 ms */
                               .fast_period = false,
                               .am_delta = 3,
                               .auto_average = false};
    CHECK_EQ(st25r3916_wakeup_start(&d, &wcfg), ST25R_OK);
    CHECK_EQ(g_rega[ST25R_REG_AMPLITUDE_MEAS_CONF], 3u << ST25R_MEAS_D_SHIFT); /* Δam, fixed ref */
    CHECK_EQ(g_rega[ST25R_REG_AMPLITUDE_MEAS_REF], 0x5C); /* calibrated to the measurement */
    CHECK_EQ(g_rega[ST25R_REG_WAKEUP_TIMER_CONTROL],
             (2u << ST25R_WUT_SHIFT) | ST25R_WUT_WAM);
    CHECK_EQ(g_rega[ST25R_REG_OP_CONTROL], ST25R_OP_WU);       /* powered down to sensing */
    CHECK(!(g_rega[ST25R_REG_OP_CONTROL] & ST25R_OP_EN));
    CHECK_EQ(g_rega[ST25R_REG_MASK_MAIN_IRQ], 0xFF);          /* only wake IRQs unmasked */
    CHECK_EQ(g_rega[ST25R_REG_MASK_ERROR_WUP_IRQ],
             (uint8_t)(0xFF & ~(ST25R_IRQ_WAM | ST25R_IRQ_WPH | ST25R_IRQ_WT)));

    /* ---- a tag enters the field: the chip raises I_wam; the host reads the cause ---- */
    g_irq |= (uint32_t)ST25R_IRQ_WAM << 16; /* error/wake-up register */
    CHECK_EQ(st25r3916_wakeup_cause(&d), ST25R_WAKE_AMPLITUDE);

    /* ---- stop restores normal interrupts and leaves wake-up mode ---- */
    st25r3916_wakeup_stop(&d);
    CHECK(!(g_rega[ST25R_REG_OP_CONTROL] & ST25R_OP_WU));
    CHECK_EQ(g_rega[ST25R_REG_MASK_ERROR_WUP_IRQ], 0x00);

    /* ---- phase + amplitude with auto-averaging: no fixed reference is written ---- */
    reset_chip();
    st25r_wakeup_cfg_t wcfg2 = {.measure_amplitude = true,
                                .measure_phase = true,
                                .period = 0,
                                .fast_period = true,
                                .am_delta = 4,
                                .ph_delta = 5,
                                .auto_average = true};
    CHECK_EQ(st25r3916_wakeup_start(&d, &wcfg2), ST25R_OK);
    CHECK_EQ(g_rega[ST25R_REG_AMPLITUDE_MEAS_CONF],
             (4u << ST25R_MEAS_D_SHIFT) | ST25R_MEAS_AE);
    CHECK_EQ(g_rega[ST25R_REG_PHASE_MEAS_CONF], (5u << ST25R_MEAS_D_SHIFT) | ST25R_MEAS_AE);
    CHECK_EQ(g_rega[ST25R_REG_AMPLITUDE_MEAS_REF], 0x00); /* auto-average: ref left alone */
    CHECK_EQ(g_rega[ST25R_REG_WAKEUP_TIMER_CONTROL],
             ST25R_WUT_WUR | ST25R_WUT_WAM | ST25R_WUT_WPH);
    g_irq |= (uint32_t)ST25R_IRQ_WPH << 16;
    CHECK_EQ(st25r3916_wakeup_cause(&d), ST25R_WAKE_PHASE);

    /* ---- parameter validation ---- */
    st25r_wakeup_cfg_t none = {.period = 1};
    CHECK_EQ(st25r3916_wakeup_start(&d, &none), ST25R_ERR_PARAM); /* no measurement selected */
    st25r_wakeup_cfg_t bigp = {.measure_amplitude = true, .period = 8};
    CHECK_EQ(st25r3916_wakeup_start(&d, &bigp), ST25R_ERR_PARAM); /* period out of range */

    /* ================= Analog configuration table ================= */
    reset_chip();
    static const st25r_analog_entry_t atbl[] = {
        /* event, reg, mask, value, spaceb */
        {ST25R_ANALOG_POWER_UP, ST25R_REG_ANT_TUNE1, 0xFF, 0x5A, 0},
        {ST25R_ANALOG_POWER_UP, ST25R_REG_ANT_TUNE2, 0x0F, 0xFF, 0}, /* masked: low nibble only */
        {ST25R_ANALOG_POLL_A, ST25R_REGB_CORR_CONF1, 0x0F, 0x03, 1}, /* space B */
        {ST25R_ANALOG_LISTEN_A, ST25R_REG_TX_DRIVER, 0xFF, 0x11, 0},
        {ST25R_ANALOG_WAKEUP, ST25R_REG_ANT_TUNE1, 0xFF, 0x77, 0},
    };
    st25r_analog_config_t acfg = {atbl, sizeof(atbl) / sizeof(atbl[0])};
    st25r3916_set_analog_config(&d, &acfg);

    /* ---- field_on applies POWER_UP and POLL_A, not LISTEN_A/WAKEUP ---- */
    CHECK_EQ(st25r3916_field_on(&d), ST25R_OK);
    CHECK_EQ(g_rega[ST25R_REG_ANT_TUNE1], 0x5A);            /* POWER_UP, space A */
    CHECK_EQ(g_rega[ST25R_REG_ANT_TUNE2], 0x0F);            /* POWER_UP, masked write */
    CHECK_EQ(g_regb[ST25R_REGB_CORR_CONF1] & 0x0F, 0x03);  /* POLL_A, space B */
    CHECK_EQ(g_rega[ST25R_REG_TX_DRIVER], 0x00);           /* LISTEN_A not applied here */

    /* ---- listen_start applies POWER_UP and LISTEN_A ---- */
    reset_chip();
    st25r3916_set_analog_config(&d, &acfg);
    st25r_nfca_target_t lt = {.nfcid1 = {1, 2, 3, 4}, .nfcid1_len = 4, .atqa = {0x44, 0}, .sak = 0};
    CHECK_EQ(st25r3916_listen_start(&d, &lt), ST25R_OK);
    CHECK_EQ(g_rega[ST25R_REG_ANT_TUNE1], 0x5A); /* POWER_UP */
    CHECK_EQ(g_rega[ST25R_REG_TX_DRIVER], 0x11); /* LISTEN_A */

    /* ---- apply by hand; and WAKEUP targets ANT_TUNE1 differently ---- */
    st25r3916_apply_analog_config(&d, ST25R_ANALOG_WAKEUP);
    CHECK_EQ(g_rega[ST25R_REG_ANT_TUNE1], 0x77);

    /* ---- clearing the table makes apply a no-op (reset defaults) ---- */
    reset_chip();
    st25r3916_set_analog_config(&d, NULL);
    st25r3916_apply_analog_config(&d, ST25R_ANALOG_POWER_UP);
    CHECK_EQ(g_rega[ST25R_REG_ANT_TUNE1], 0x00);

    TEST_MAIN_END();
}
