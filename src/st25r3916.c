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

void st25r3916_set_analog_config(st25r_t *d, const st25r_analog_config_t *cfg)
{
    d->analog = cfg;
}

void st25r3916_apply_analog_config(st25r_t *d, st25r_analog_event_t event)
{
    if (!d->analog || !d->analog->entries) {
        return;
    }
    for (size_t i = 0; i < d->analog->count; i++) {
        const st25r_analog_entry_t *e = &d->analog->entries[i];
        if (e->event != (uint8_t)event) {
            continue;
        }
        if (e->spaceb) {
            uint8_t v = st25r3916_read_regb(d, e->reg);
            st25r3916_write_regb(d, e->reg, (uint8_t)((v & ~e->mask) | (e->value & e->mask)));
        } else {
            st25r3916_modify_reg(d, e->reg, e->mask, e->value);
        }
    }
}

/* ===== RF: power-up and ISO 14443-A reader (DS13541 §4.2.13) ============================
 * UNTESTED ON HARDWARE. See the header for the contract and caveats. */

#include <string.h>

/* Pack an interrupt bit into the 32-bit value st25r3916_get_interrupts() returns:
 * M = main (0x1A), T = timer/NFC (0x1B), E = error/wake-up (0x1C). */
#define IRQ_M(bit) ((uint32_t)(bit))
#define IRQ_T(bit) ((uint32_t)(bit) << 8)
#define IRQ_E(bit) ((uint32_t)(bit) << 16)

/* Wait up to `timeout_ms` for any interrupt in `want` (packed). Returns the accumulated
 * interrupts that were seen (so the caller can tell which fired), or 0 on timeout. */
static uint32_t wait_irqs(st25r_t *d, uint32_t want, int timeout_ms)
{
    uint32_t got = 0;
    int remaining = timeout_ms > 0 ? timeout_ms : 1;
    while (remaining > 0) {
        if (d->bus->wait_irq) {
            bool fired = d->bus->wait_irq(d->bus->ctx, remaining);
            got |= st25r3916_get_interrupts(d);
            if (got & want) {
                return got;
            }
            if (!fired) {
                break; /* the IRQ wait timed out */
            }
            remaining--; /* fired for another reason; keep waiting, bounded */
        } else {
            if (d->bus->delay_us) {
                d->bus->delay_us(d->bus->ctx, 1000);
            }
            got |= st25r3916_get_interrupts(d);
            if (got & want) {
                return got;
            }
            remaining--;
        }
    }
    return 0;
}

int st25r3916_field_on(st25r_t *d)
{
    st25r3916_cmd(d, ST25R_CMD_SET_DEFAULT);
    if (d->bus->delay_us) {
        d->bus->delay_us(d->bus->ctx, 1000);
    }

    /* Ready mode: enable the oscillator and regulators, then wait for the crystal to settle
     * (osc_ok in the auxiliary display register; DS13541 §4.2.13). */
    st25r3916_modify_reg(d, ST25R_REG_OP_CONTROL, ST25R_OP_EN, ST25R_OP_EN);
    int tries = 0;
    while (!(st25r3916_read_reg(d, ST25R_REG_AUX_DISPLAY) & ST25R_AUX_OSC_OK)) {
        if (d->bus->delay_us) {
            d->bus->delay_us(d->bus->ctx, 100);
        }
        if (++tries > 100) { /* ~10 ms */
            return ST25R_ERR_TIMEOUT;
        }
    }
    st25r3916_apply_analog_config(d, ST25R_ANALOG_POWER_UP);

    /* NFC-A reader at 106 kbit/s: initiator, ISO14443A, standard framing. */
    st25r3916_write_reg(d, ST25R_REG_MODE, (uint8_t)(ST25R_OM_ISO14443A << ST25R_MODE_OM_SHIFT));
    st25r3916_write_reg(d, ST25R_REG_BIT_RATE,
                        (uint8_t)((ST25R_BR_106 << ST25R_BR_TX_SHIFT) | (ST25R_BR_106 << ST25R_BR_RX_SHIFT)));

    /* Match the regulated voltages to the supply. */
    st25r3916_get_interrupts(d);
    st25r3916_cmd(d, ST25R_CMD_ADJUST_REGULATORS);
    wait_irqs(d, IRQ_T(ST25R_IRQ_DCT), 10);

    /* Enable the receiver and transmitter. */
    st25r3916_modify_reg(d, ST25R_REG_OP_CONTROL, ST25R_OP_RX_EN | ST25R_OP_TX_EN,
                         ST25R_OP_RX_EN | ST25R_OP_TX_EN);
    st25r3916_apply_analog_config(d, ST25R_ANALOG_POLL_A); /* Rx/Tx analog for the antenna */

    /* Switch the field on with initial RF collision avoidance (does not transmit if another
     * reader's field is already present). I_cat means the field is on and the minimum guard
     * time has started; I_cac means an external field was detected and ours stayed off. */
    st25r3916_get_interrupts(d);
    st25r3916_cmd(d, ST25R_CMD_INITIAL_FIELD_ON);
    uint32_t irq = wait_irqs(d, IRQ_T(ST25R_IRQ_CAT) | IRQ_T(ST25R_IRQ_CAC), 15);
    if (irq & IRQ_T(ST25R_IRQ_CAC)) {
        return ST25R_ERR_COLLISION;
    }
    if (!(irq & IRQ_T(ST25R_IRQ_CAT))) {
        return ST25R_ERR_TIMEOUT;
    }
    /* ISO 14443 guard time: the field must be on a while before the first command (~5 ms). */
    if (d->bus->delay_us) {
        d->bus->delay_us(d->bus->ctx, 5000);
    }
    return ST25R_OK;
}

void st25r3916_field_off(st25r_t *d)
{
    st25r3916_cmd(d, ST25R_CMD_STOP);
    /* Drop the transmitter and the oscillator/regulators to idle. */
    st25r3916_modify_reg(d, ST25R_REG_OP_CONTROL,
                         ST25R_OP_EN | ST25R_OP_RX_EN | ST25R_OP_TX_EN, 0);
}

int st25r3916_transceive(st25r_t *d, const uint8_t *tx, int tx_bits, unsigned flags,
                         uint8_t *rx, int rx_cap, int *rx_bits, int timeout_ms)
{
    if (tx_bits < 0) {
        return ST25R_ERR_PARAM;
    }
    int nbytes = tx_bits / 8;
    int nbits = tx_bits % 8;
    int fifo_bytes = nbytes + (nbits ? 1 : 0);
    if (fifo_bytes > ST25R_FIFO_DEPTH) {
        return ST25R_ERR_PARAM;
    }

    st25r3916_cmd(d, ST25R_CMD_STOP);
    st25r3916_cmd(d, ST25R_CMD_RESET_RXGAIN);

    /* Framing: anticollision bit-orientation and parity handling. */
    uint8_t a_mask = ST25R_A_ANTCL | ST25R_A_NO_TX_PAR | ST25R_A_NO_RX_PAR;
    uint8_t a_val = 0;
    if (flags & ST25R_TX_ANTICOL) {
        a_val |= ST25R_A_ANTCL;
    }
    if (flags & ST25R_TX_NO_PARITY) {
        a_val |= ST25R_A_NO_TX_PAR | ST25R_A_NO_RX_PAR;
    }
    st25r3916_modify_reg(d, ST25R_REG_ISO14443A_NFC, a_mask, a_val);

    st25r3916_write_reg(d, ST25R_REG_NUM_TX_BYTES1, ST25R_NTX1(nbytes));
    st25r3916_write_reg(d, ST25R_REG_NUM_TX_BYTES2, ST25R_NTX2(nbytes, nbits));
    if (fifo_bytes > 0) {
        st25r3916_write_fifo(d, tx, (size_t)fifo_bytes);
    }

    st25r3916_get_interrupts(d);
    st25r3916_cmd(d, (flags & ST25R_TX_CRC) ? ST25R_CMD_TRANSMIT_WITH_CRC
                                            : ST25R_CMD_TRANSMIT_WITHOUT_CRC);
    if (!(wait_irqs(d, IRQ_M(ST25R_IRQ_TXE), timeout_ms) & IRQ_M(ST25R_IRQ_TXE))) {
        return ST25R_ERR_TIMEOUT;
    }

    uint32_t want = IRQ_M(ST25R_IRQ_RXE) | IRQ_M(ST25R_IRQ_COL) | IRQ_T(ST25R_IRQ_NRE) |
                    IRQ_E(ST25R_IRQ_CRC | ST25R_IRQ_PAR | ST25R_IRQ_ERR1 | ST25R_IRQ_ERR2);
    uint32_t irq = wait_irqs(d, want, timeout_ms);
    if (irq & IRQ_M(ST25R_IRQ_COL)) {
        return ST25R_ERR_COLLISION;
    }
    if (irq & IRQ_E(ST25R_IRQ_CRC | ST25R_IRQ_PAR)) {
        return ST25R_ERR_CRC;
    }
    if (irq & IRQ_E(ST25R_IRQ_ERR1 | ST25R_IRQ_ERR2)) {
        return ST25R_ERR_FRAMING;
    }
    if (!(irq & IRQ_M(ST25R_IRQ_RXE))) {
        return ST25R_ERR_TIMEOUT;
    }

    uint8_t fs[2];
    st25r3916_read_regs(d, ST25R_REG_FIFO_STATUS1, fs, 2); /* read the count before emptying */
    int count = fs[0] | ((fs[1] >> ST25R_FIFO2_B9_B8_SHIFT) << 8);
    int lb = (fs[1] & ST25R_FIFO2_LB_MASK) >> ST25R_FIFO2_LB_SHIFT;
    if (count > rx_cap) {
        return ST25R_ERR_NOMEM;
    }
    if (count > 0) {
        st25r3916_read_fifo(d, rx, (size_t)count);
    }
    if (rx_bits) {
        *rx_bits = (lb && count) ? (count - 1) * 8 + lb : count * 8;
    }
    return count;
}

int st25r3916_nfca_request(st25r_t *d, uint8_t atqa[2], bool wupa)
{
    st25r3916_cmd(d, ST25R_CMD_STOP);
    st25r3916_cmd(d, ST25R_CMD_RESET_RXGAIN);
    /* Standard framing for the short frame (no anticollision, parity on). */
    st25r3916_modify_reg(d, ST25R_REG_ISO14443A_NFC,
                         ST25R_A_ANTCL | ST25R_A_NO_TX_PAR | ST25R_A_NO_RX_PAR, 0);
    st25r3916_write_reg(d, ST25R_REG_NUM_TX_BYTES1, 0);
    st25r3916_write_reg(d, ST25R_REG_NUM_TX_BYTES2, 0);

    st25r3916_get_interrupts(d);
    st25r3916_cmd(d, wupa ? ST25R_CMD_TRANSMIT_WUPA : ST25R_CMD_TRANSMIT_REQA);
    if (!(wait_irqs(d, IRQ_M(ST25R_IRQ_TXE), 10) & IRQ_M(ST25R_IRQ_TXE))) {
        return ST25R_ERR_TIMEOUT;
    }
    uint32_t irq = wait_irqs(d, IRQ_M(ST25R_IRQ_RXE) | IRQ_M(ST25R_IRQ_COL) | IRQ_T(ST25R_IRQ_NRE), 10);
    if (!(irq & (IRQ_M(ST25R_IRQ_RXE) | IRQ_M(ST25R_IRQ_COL)))) {
        return ST25R_ERR_TIMEOUT; /* no card answered */
    }
    uint8_t fs[2];
    st25r3916_read_regs(d, ST25R_REG_FIFO_STATUS1, fs, 2);
    int count = fs[0] | ((fs[1] >> ST25R_FIFO2_B9_B8_SHIFT) << 8);
    if (count < 2) {
        return ST25R_ERR_FRAMING;
    }
    uint8_t buf[2];
    st25r3916_read_fifo(d, buf, 2);
    atqa[0] = buf[0];
    atqa[1] = buf[1];
    return ST25R_OK;
}

int st25r3916_nfca_select(st25r_t *d, st25r_nfca_t *out)
{
    static const uint8_t SEL[3] = {0x93, 0x95, 0x97}; /* cascade levels 1..3 */
    memset(out, 0, sizeof(*out));

    int rc = st25r3916_nfca_request(d, out->atqa, false);
    if (rc != ST25R_OK) {
        return rc;
    }

    for (int lvl = 0; lvl < 3; lvl++) {
        /* ANTICOLLISION: SEL + NVB=0x20 (no UID bits known), bit-oriented, no CRC.
         * A single card answers with its 4 UID bytes of this level plus the BCC. */
        uint8_t ac[2] = {SEL[lvl], 0x20};
        uint8_t resp[8];
        rc = st25r3916_transceive(d, ac, 16, ST25R_TX_ANTICOL, resp, sizeof(resp), NULL, 20);
        if (rc == ST25R_ERR_COLLISION) {
            return ST25R_ERR_COLLISION; /* more than one card; not resolved here */
        }
        if (rc < 0) {
            return rc;
        }
        if (rc < 5) {
            return ST25R_ERR_FRAMING;
        }
        uint8_t bcc = (uint8_t)(resp[0] ^ resp[1] ^ resp[2] ^ resp[3]);
        if (bcc != resp[4]) {
            return ST25R_ERR_FRAMING;
        }

        /* SELECT: SEL + NVB=0x70 + the 4 UID bytes + BCC, with CRC. The card returns SAK. */
        uint8_t sel[7] = {SEL[lvl], 0x70, resp[0], resp[1], resp[2], resp[3], resp[4]};
        uint8_t sak[4];
        rc = st25r3916_transceive(d, sel, 56, ST25R_TX_CRC, sak, sizeof(sak), NULL, 20);
        if (rc < 1) {
            return rc < 0 ? rc : ST25R_ERR_FRAMING;
        }
        out->sak = sak[0];

        /* Accumulate the UID. A level that carries the cascade tag (CT=0x88) contributes its
         * last three bytes; a final level contributes all four. */
        if (resp[0] == 0x88) {
            if (out->uid_len + 3 <= (int)sizeof(out->uid)) {
                out->uid[out->uid_len++] = resp[1];
                out->uid[out->uid_len++] = resp[2];
                out->uid[out->uid_len++] = resp[3];
            }
        } else {
            if (out->uid_len + 4 <= (int)sizeof(out->uid)) {
                out->uid[out->uid_len++] = resp[0];
                out->uid[out->uid_len++] = resp[1];
                out->uid[out->uid_len++] = resp[2];
                out->uid[out->uid_len++] = resp[3];
            }
        }

        if (!(sak[0] & 0x04)) { /* cascade complete (SAK cascade bit clear) */
            return ST25R_OK;
        }
    }
    return ST25R_ERR_FRAMING; /* ran out of cascade levels without completing */
}

/* ===== RF: ISO 14443-A card emulation (listen / target mode, DS13541 §4.2.14) ==========
 * UNTESTED ON HARDWARE. See the header for the contract and caveats. */

/* Load PT_Memory from location 0 (NFC-A config area) via the dedicated SPI operation. */
static void load_pt_memory_a(st25r_t *d, const uint8_t *data, size_t n)
{
    uint8_t buf[1 + ST25R_PT_NFCA_LEN];
    if (n > ST25R_PT_NFCA_LEN) {
        n = ST25R_PT_NFCA_LEN;
    }
    buf[0] = ST25R_SPI_PT_LOAD_A;
    memcpy(&buf[1], data, n);
    d->bus->txn(d->bus->ctx, buf, NULL, 1 + n);
}

int st25r3916_listen_start(st25r_t *d, const st25r_nfca_target_t *t)
{
    if (!t || (t->nfcid1_len != 4 && t->nfcid1_len != 7)) {
        return ST25R_ERR_PARAM;
    }

    st25r3916_cmd(d, ST25R_CMD_SET_DEFAULT);
    if (d->bus->delay_us) {
        d->bus->delay_us(d->bus->ctx, 1000);
    }

    /* Enable the oscillator/regulators (needed to load-modulate replies) and wait for it. */
    st25r3916_modify_reg(d, ST25R_REG_OP_CONTROL, ST25R_OP_EN, ST25R_OP_EN);
    int tries = 0;
    while (!(st25r3916_read_reg(d, ST25R_REG_AUX_DISPLAY) & ST25R_AUX_OSC_OK)) {
        if (d->bus->delay_us) {
            d->bus->delay_us(d->bus->ctx, 100);
        }
        if (++tries > 100) {
            return ST25R_ERR_TIMEOUT;
        }
    }
    st25r3916_apply_analog_config(d, ST25R_ANALOG_POWER_UP);

    /* PT_Memory: NFCID1, SENS_RES (ATQA) and the SEL_RES (SAK) for each cascade level. For a
     * 7-byte UID, level 1's SAK carries the cascade bit and level 2's is the final SAK. */
    uint8_t pt[ST25R_PT_NFCA_LEN];
    memset(pt, 0, sizeof(pt));
    memcpy(&pt[ST25R_PT_OFF_NFCID1], t->nfcid1, t->nfcid1_len);
    pt[ST25R_PT_OFF_SENS_RES] = t->atqa[0];
    pt[ST25R_PT_OFF_SENS_RES + 1] = t->atqa[1];
    if (t->nfcid1_len == 4) {
        pt[ST25R_PT_OFF_SELR_L1] = t->sak;
    } else {
        pt[ST25R_PT_OFF_SELR_L1] = (uint8_t)(t->sak | 0x04); /* cascade bit */
        pt[ST25R_PT_OFF_SELR_L2] = t->sak;
    }
    load_pt_memory_a(d, pt, sizeof(pt));

    /* Automatic NFC-A anticollision with the recommended FDT compensation (fdel = 2). */
    st25r3916_write_reg(d, ST25R_REG_PASSIVE_TARGET, (uint8_t)(2u << ST25R_PT_FDEL_SHIFT));
    /* NFC-A passive target mode at 106 kbit/s. */
    st25r3916_write_reg(d, ST25R_REG_MODE,
                        (uint8_t)(ST25R_MODE_TARG | (ST25R_OM_ISO14443A << ST25R_MODE_OM_SHIFT)));
    st25r3916_write_reg(d, ST25R_REG_BIT_RATE,
                        (uint8_t)((ST25R_BR_106 << ST25R_BR_TX_SHIFT) | (ST25R_BR_106 << ST25R_BR_RX_SHIFT)));
    /* Receiver on, automatic external field detector (required for passive target). */
    st25r3916_modify_reg(d, ST25R_REG_OP_CONTROL, ST25R_OP_RX_EN | ST25R_OP_EN_FD_MASK,
                         ST25R_OP_RX_EN | ST25R_OP_EN_FD_AUTO);
    st25r3916_apply_analog_config(d, ST25R_ANALOG_LISTEN_A);

    /* Go to the Sense (idle) state to wait for a reader's REQA. */
    st25r3916_get_interrupts(d);
    st25r3916_cmd(d, ST25R_CMD_GOTO_SENSE);
    return ST25R_OK;
}

void st25r3916_listen_stop(st25r_t *d)
{
    st25r3916_cmd(d, ST25R_CMD_STOP);
    st25r3916_write_reg(d, ST25R_REG_MODE, 0); /* back to initiator */
    st25r3916_modify_reg(d, ST25R_REG_OP_CONTROL,
                         ST25R_OP_EN | ST25R_OP_RX_EN | ST25R_OP_TX_EN | ST25R_OP_EN_FD_MASK, 0);
}

st25r_listen_event_t st25r3916_listen_poll(st25r_t *d, uint8_t *rx, int rx_cap, int *rx_len,
                                           int timeout_ms)
{
    uint32_t want = IRQ_M(ST25R_IRQ_RXE) | ((uint32_t)(ST25R_IRQ_WU_A | ST25R_IRQ_RXE_PTA) << 24) |
                    IRQ_T(ST25R_IRQ_EOF);
    uint32_t irq = wait_irqs(d, want, timeout_ms);

    if (irq & IRQ_T(ST25R_IRQ_EOF)) {
        return ST25R_LISTEN_FIELD_OFF; /* the reader's field dropped */
    }
    /* A received higher-layer frame (I_rxe, or I_rxe_pta after the auto anticollision). */
    if ((irq & IRQ_M(ST25R_IRQ_RXE)) || (irq & ((uint32_t)ST25R_IRQ_RXE_PTA << 24))) {
        uint8_t fs[2];
        st25r3916_read_regs(d, ST25R_REG_FIFO_STATUS1, fs, 2);
        int count = fs[0] | ((fs[1] >> ST25R_FIFO2_B9_B8_SHIFT) << 8);
        if (count > 0 && rx) {
            if (count > rx_cap) {
                count = rx_cap;
            }
            st25r3916_read_fifo(d, rx, (size_t)count);
        }
        if (rx_len) {
            *rx_len = count;
        }
        if (count > 0) {
            return ST25R_LISTEN_RECEIVED;
        }
    }
    if (irq & ((uint32_t)ST25R_IRQ_WU_A << 24)) {
        return ST25R_LISTEN_ACTIVATED; /* selected, no higher-layer frame yet */
    }
    return ST25R_LISTEN_NONE;
}

int st25r3916_listen_respond(st25r_t *d, const uint8_t *tx, int tx_len, bool with_crc,
                             int timeout_ms)
{
    if (tx_len < 0 || tx_len > ST25R_FIFO_DEPTH) {
        return ST25R_ERR_PARAM;
    }
    st25r3916_write_reg(d, ST25R_REG_NUM_TX_BYTES1, ST25R_NTX1(tx_len));
    st25r3916_write_reg(d, ST25R_REG_NUM_TX_BYTES2, ST25R_NTX2(tx_len, 0));
    if (tx_len > 0) {
        st25r3916_write_fifo(d, tx, (size_t)tx_len);
    }
    st25r3916_get_interrupts(d);
    st25r3916_cmd(d, with_crc ? ST25R_CMD_TRANSMIT_WITH_CRC : ST25R_CMD_TRANSMIT_WITHOUT_CRC);
    if (!(wait_irqs(d, IRQ_M(ST25R_IRQ_TXE), timeout_ms) & IRQ_M(ST25R_IRQ_TXE))) {
        return ST25R_ERR_TIMEOUT;
    }
    return ST25R_OK;
}

/* ===== RF: wake-up mode (low-power tag detection, DS13541 §4.5.67-69) ==================
 * UNTESTED ON HARDWARE. See the header for the contract and caveats. */

/* Enable the oscillator and wait for it to stabilise (shared by the RF entry points). */
static int wait_ready(st25r_t *d)
{
    st25r3916_modify_reg(d, ST25R_REG_OP_CONTROL, ST25R_OP_EN, ST25R_OP_EN);
    for (int tries = 0; tries <= 100; tries++) {
        if (st25r3916_read_reg(d, ST25R_REG_AUX_DISPLAY) & ST25R_AUX_OSC_OK) {
            return ST25R_OK;
        }
        if (d->bus->delay_us) {
            d->bus->delay_us(d->bus->ctx, 100);
        }
    }
    return ST25R_ERR_TIMEOUT;
}

/* Take one measurement with `measure_cmd` and copy the result register into the reference
 * register, so a later wake-up compares against the field as it is right now (no tag). */
static void calibrate_ref(st25r_t *d, uint8_t measure_cmd, uint8_t display_reg, uint8_t ref_reg)
{
    st25r3916_get_interrupts(d);
    st25r3916_cmd(d, measure_cmd);
    wait_irqs(d, IRQ_T(ST25R_IRQ_DCT), 5);
    st25r3916_write_reg(d, ref_reg, st25r3916_read_reg(d, display_reg));
}

int st25r3916_wakeup_start(st25r_t *d, const st25r_wakeup_cfg_t *cfg)
{
    if (!cfg || (!cfg->measure_amplitude && !cfg->measure_phase) ||
        cfg->period > 7 || cfg->am_delta > 15 || cfg->ph_delta > 15) {
        return ST25R_ERR_PARAM;
    }

    int rc = wait_ready(d);
    if (rc != ST25R_OK) {
        return rc;
    }
    st25r3916_apply_analog_config(d, ST25R_ANALOG_POWER_UP);

    if (cfg->measure_amplitude) {
        uint8_t conf = (uint8_t)((cfg->am_delta << ST25R_MEAS_D_SHIFT) |
                                 (cfg->auto_average ? ST25R_MEAS_AE : 0));
        st25r3916_write_reg(d, ST25R_REG_AMPLITUDE_MEAS_CONF, conf);
        if (!cfg->auto_average) {
            calibrate_ref(d, ST25R_CMD_MEASURE_AMPLITUDE, ST25R_REG_AMPLITUDE_MEAS_DISPLAY,
                          ST25R_REG_AMPLITUDE_MEAS_REF);
        }
    }
    if (cfg->measure_phase) {
        uint8_t conf = (uint8_t)((cfg->ph_delta << ST25R_MEAS_D_SHIFT) |
                                 (cfg->auto_average ? ST25R_MEAS_AE : 0));
        st25r3916_write_reg(d, ST25R_REG_PHASE_MEAS_CONF, conf);
        if (!cfg->auto_average) {
            calibrate_ref(d, ST25R_CMD_MEASURE_PHASE, ST25R_REG_PHASE_MEAS_DISPLAY,
                          ST25R_REG_PHASE_MEAS_REF);
        }
    }

    uint8_t wut = (uint8_t)((cfg->fast_period ? ST25R_WUT_WUR : 0) |
                            ((cfg->period & 0x7) << ST25R_WUT_SHIFT) |
                            (cfg->measure_amplitude ? ST25R_WUT_WAM : 0) |
                            (cfg->measure_phase ? ST25R_WUT_WPH : 0));
    st25r3916_write_reg(d, ST25R_REG_WAKEUP_TIMER_CONTROL, wut);
    st25r3916_apply_analog_config(d, ST25R_ANALOG_WAKEUP);

    /* Let only the wake sources (I_wam / I_wph / I_wt) reach the IRQ pin. */
    uint32_t keep = ST25R_IRQ_WAM | ST25R_IRQ_WPH | ST25R_IRQ_WT;
    uint32_t mask = 0xFFu | (0xFFu << 8) | ((uint32_t)(0xFFu & ~keep) << 16) | (0xFFu << 24);
    st25r3916_set_interrupt_mask(d, mask);

    /* Power down to periodic sensing: wake-up mode on, oscillator/receiver/transmitter off. */
    st25r3916_get_interrupts(d);
    st25r3916_write_reg(d, ST25R_REG_OP_CONTROL, ST25R_OP_WU);
    return ST25R_OK;
}

int st25r3916_wakeup_cause(st25r_t *d)
{
    uint8_t e = ST25R_IRQ_ERROR_WUP(st25r3916_get_interrupts(d));
    int flags = 0;
    if (e & ST25R_IRQ_WT) {
        flags |= ST25R_WAKE_TIMER;
    }
    if (e & ST25R_IRQ_WAM) {
        flags |= ST25R_WAKE_AMPLITUDE;
    }
    if (e & ST25R_IRQ_WPH) {
        flags |= ST25R_WAKE_PHASE;
    }
    return flags;
}

void st25r3916_wakeup_stop(st25r_t *d)
{
    st25r3916_cmd(d, ST25R_CMD_STOP);
    st25r3916_modify_reg(d, ST25R_REG_OP_CONTROL, ST25R_OP_WU | ST25R_OP_EN, 0);
    st25r3916_set_interrupt_mask(d, 0); /* default: nothing masked */
}
