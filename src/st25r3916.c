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

int st25r3916_set_tech(st25r_t *d, st25r_tech_t tech, uint8_t br_tx, uint8_t br_rx)
{
    uint8_t om;
    uint8_t am = 0;
    st25r_analog_event_t ev;
    if (br_tx > ST25R_BR_848 || br_rx > ST25R_BR_848) {
        return ST25R_ERR_PARAM;
    }
    switch (tech) {
    case ST25R_TECH_A:
        om = ST25R_OM_ISO14443A;
        ev = ST25R_ANALOG_POLL_A;
        break;
    case ST25R_TECH_B:
        om = ST25R_OM_ISO14443B;
        am = ST25R_MODE_TR_AM; /* ISO 14443-B is 10 % ASK (DS13541 §4.2.1) */
        ev = ST25R_ANALOG_POLL_B;
        break;
    case ST25R_TECH_F:
        if (br_tx == ST25R_BR_106 || br_tx == ST25R_BR_848 || br_rx == ST25R_BR_106 ||
            br_rx == ST25R_BR_848) {
            return ST25R_ERR_PARAM; /* FeliCa runs at 212 and 424 kbit/s only */
        }
        om = ST25R_OM_FELICA;
        am = ST25R_MODE_TR_AM;
        ev = ST25R_ANALOG_POLL_F;
        break;
    case ST25R_TECH_V:
        om = ST25R_OM_SUBC_STREAM; /* the stream register sets the timing, not 0x04 */
        am = d->nfcv_am ? ST25R_MODE_TR_AM : 0;
        br_tx = br_rx = ST25R_BR_106;
        ev = ST25R_ANALOG_POLL_V;
        break;
    default:
        return ST25R_ERR_PARAM;
    }
    st25r3916_write_reg(d, ST25R_REG_MODE, (uint8_t)((om << ST25R_MODE_OM_SHIFT) | am));
    st25r3916_write_reg(d, ST25R_REG_BIT_RATE,
                        (uint8_t)((br_tx << ST25R_BR_TX_SHIFT) | (br_rx << ST25R_BR_RX_SHIFT)));
    if (tech == ST25R_TECH_B || tech == ST25R_TECH_F) {
        st25r3916_write_reg(d, ST25R_REG_ISO14443B_1, 0x00);
        st25r3916_write_reg(d, ST25R_REG_ISO14443B_FELICA, 0x00);
    }
    if (tech == ST25R_TECH_V) {
        /* 424 kHz subcarrier, 8 pulses (half an ISO 15693 bit) per report bit; Tx slots of
         * fc/128. The driver frames and checks the CRC itself. */
        st25r3916_write_reg(d, ST25R_REG_STREAM_MODE,
                            (uint8_t)((ST25R_STREAM_SCF_424 << ST25R_STREAM_SCF_SHIFT) |
                                      (ST25R_STREAM_SCP_8 << ST25R_STREAM_SCP_SHIFT) |
                                      (ST25R_STREAM_STX_128 << ST25R_STREAM_STX_SHIFT)));
        st25r3916_modify_reg(d, ST25R_REG_AUX, ST25R_AUXDEF_NO_CRC_RX, ST25R_AUXDEF_NO_CRC_RX);
    } else {
        st25r3916_modify_reg(d, ST25R_REG_AUX, ST25R_AUXDEF_NO_CRC_RX, 0);
    }
    d->tech = (uint8_t)tech;
    st25r3916_apply_analog_config(d, ev);
    return ST25R_OK;
}

int st25r3916_field_on(st25r_t *d)
{
    return st25r3916_field_on_tech(d, ST25R_TECH_A, ST25R_BR_106);
}

int st25r3916_field_on_tech(st25r_t *d, st25r_tech_t tech, uint8_t br)
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

    /* Reader (initiator) in the chosen protocol and bit rate. */
    int rc = st25r3916_set_tech(d, tech, br, br);
    if (rc != ST25R_OK) {
        return rc;
    }

    /* Match the regulated voltages to the supply. */
    st25r3916_get_interrupts(d);
    st25r3916_cmd(d, ST25R_CMD_ADJUST_REGULATORS);
    wait_irqs(d, IRQ_T(ST25R_IRQ_DCT), 10);

    /* Enable the receiver and transmitter. */
    st25r3916_modify_reg(d, ST25R_REG_OP_CONTROL, ST25R_OP_RX_EN | ST25R_OP_TX_EN,
                         ST25R_OP_RX_EN | ST25R_OP_TX_EN);

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

static int nfcv_xfer(st25r_t *d, const uint8_t *req, int req_len, bool crc, uint8_t *resp,
                     int resp_cap, int timeout_ms);
static int nfcf_xfer(st25r_t *d, const uint8_t *tx, int len, uint8_t *rx, int rx_cap,
                     int timeout_ms);

/* The framed transceive in the chip's own framing (A, B, F). */
static int xfer(st25r_t *d, const uint8_t *tx, int tx_bits, unsigned flags, uint8_t *rx,
                int rx_cap, int *rx_bits, int timeout_ms)
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

int st25r3916_transceive(st25r_t *d, const uint8_t *tx, int tx_bits, unsigned flags,
                         uint8_t *rx, int rx_cap, int *rx_bits, int timeout_ms)
{
    if (d->tech == ST25R_TECH_V || d->tech == ST25R_TECH_F) {
        if (tx_bits < 0 || tx_bits % 8) {
            return ST25R_ERR_PARAM; /* whole bytes only */
        }
        int n = d->tech == ST25R_TECH_V
                    ? nfcv_xfer(d, tx, tx_bits / 8, (flags & ST25R_TX_CRC) != 0, rx, rx_cap, timeout_ms)
                    : nfcf_xfer(d, tx, tx_bits / 8, rx, rx_cap, timeout_ms);
        if (n >= 0 && rx_bits) {
            *rx_bits = n * 8;
        }
        return n;
    }
    return xfer(d, tx, tx_bits, flags, rx, rx_cap, rx_bits, timeout_ms);
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

/* One ISO 14443-3 anticollision round. `frame` is SEL, NVB and the 5 bytes CLn (4 UID bytes
 * + BCC) of this cascade level, of which the first `known` bits (0..39) are valid. Sends SEL,
 * NVB and those bits as a bit-oriented frame, merges the card(s)' answer into frame[2..6] and
 * returns the number of valid bits after it: 40 for a clean answer, or the position of the
 * first collided bit (that bit and the ones after it are left 0). Negative ST25R_ERR_* on
 * failure.
 *
 * Two points DS13541 does not spell out, to verify on hardware: (1) the first received byte
 * is assumed to be aligned to the split position, i.e. the card's first bit lands at bit
 * `nbtx` of FIFO byte 0 (as ISO 14443-3 Figure "split byte" draws it); (2) c_byte/c_bit in
 * the Collision display register are assumed to count from the start of the frame (SEL),
 * since Table 68 says the register "covers the ISO14443A anticollision command". Either
 * being wrong shows up as ST25R_ERR_FRAMING from the position check below, not a wrong UID
 * (the BCC check and SELECT catch the rest). */
static int anticol_round(st25r_t *d, uint8_t frame[7], int known)
{
    if (known < 0 || known >= 40) {
        return ST25R_ERR_PARAM;
    }
    int tx_bits = 16 + known;
    int nbytes = tx_bits / 8;
    int nbits = tx_bits % 8;
    frame[1] = (uint8_t)((nbytes << 4) | nbits); /* NVB: bytes (incl. SEL/NVB), then bits */
    uint8_t keep = (uint8_t)((1u << nbits) - 1u); /* valid low bits of the split byte */
    if (nbits) {
        frame[nbytes] &= keep;
    }
    for (int i = nbytes + (nbits ? 1 : 0); i < 7; i++) {
        frame[i] = 0;
    }

    st25r3916_cmd(d, ST25R_CMD_STOP);
    st25r3916_cmd(d, ST25R_CMD_RESET_RXGAIN);
    st25r3916_modify_reg(d, ST25R_REG_ISO14443A_NFC,
                         ST25R_A_ANTCL | ST25R_A_NO_TX_PAR | ST25R_A_NO_RX_PAR, ST25R_A_ANTCL);
    st25r3916_write_reg(d, ST25R_REG_NUM_TX_BYTES1, ST25R_NTX1(nbytes));
    st25r3916_write_reg(d, ST25R_REG_NUM_TX_BYTES2, ST25R_NTX2(nbytes, nbits));
    st25r3916_write_fifo(d, frame, (size_t)(nbytes + (nbits ? 1 : 0)));

    st25r3916_get_interrupts(d);
    st25r3916_cmd(d, ST25R_CMD_TRANSMIT_WITHOUT_CRC);
    if (!(wait_irqs(d, IRQ_M(ST25R_IRQ_TXE), 20) & IRQ_M(ST25R_IRQ_TXE))) {
        return ST25R_ERR_TIMEOUT;
    }
    uint32_t end = IRQ_M(ST25R_IRQ_RXE) | IRQ_T(ST25R_IRQ_NRE);
    uint32_t irq = wait_irqs(d, end | IRQ_M(ST25R_IRQ_COL) | IRQ_E(ST25R_IRQ_ERR1), 20);
    if ((irq & IRQ_M(ST25R_IRQ_COL)) && !(irq & end)) {
        irq |= wait_irqs(d, end, 20); /* collision seen; let the reception finish */
    }
    if (irq & IRQ_E(ST25R_IRQ_ERR1)) {
        return ST25R_ERR_FRAMING; /* the collision register is not valid after I_err1 */
    }
    if (!(irq & (IRQ_M(ST25R_IRQ_RXE) | IRQ_M(ST25R_IRQ_COL)))) {
        return ST25R_ERR_TIMEOUT; /* no card answered */
    }

    uint8_t fs[2];
    st25r3916_read_regs(d, ST25R_REG_FIFO_STATUS1, fs, 2);
    int count = fs[0] | ((fs[1] >> ST25R_FIFO2_B9_B8_SHIFT) << 8);
    uint8_t rx[8] = {0};
    int room = 7 - nbytes; /* frame bytes still to fill, the split byte included */
    if (count > (int)sizeof(rx)) {
        return ST25R_ERR_FRAMING;
    }
    if (count > 0) {
        st25r3916_read_fifo(d, rx, (size_t)count);
    }
    for (int i = 0; i < count && i < room; i++) {
        if (i == 0 && nbits) {
            frame[nbytes] = (uint8_t)((frame[nbytes] & keep) | (rx[0] & (uint8_t)~keep));
        } else {
            frame[nbytes + i] = rx[i];
        }
    }

    if (!(irq & IRQ_M(ST25R_IRQ_COL))) {
        return count >= room ? 40 : ST25R_ERR_FRAMING;
    }
    uint8_t coll = st25r3916_read_reg(d, ST25R_REG_COLLISION);
    if (coll & ST25R_COLL_PB) {
        return ST25R_ERR_FRAMING; /* equal data bits but differing parity: noise, not cards */
    }
    int pos = (int)((coll & ST25R_COLL_BYTE_MASK) >> ST25R_COLL_BYTE_SHIFT) * 8 +
              (int)((coll & ST25R_COLL_BIT_MASK) >> ST25R_COLL_BIT_SHIFT) - 16;
    if (pos < known || pos >= 40) {
        return ST25R_ERR_FRAMING;
    }
    /* Keep the bits before the collision; clear it and everything after it. */
    frame[2 + pos / 8] &= (uint8_t)((1u << (pos % 8)) - 1u);
    for (int i = 2 + pos / 8 + 1; i < 7; i++) {
        frame[i] = 0;
    }
    return pos;
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
        /* ANTICOLLISION (ISO 14443-3 §6.5.3): start with no UID bits known (NVB=0x20). On a
         * collision keep the bits before it, choose 1 for the collided bit and ask again
         * with the longer prefix; only the cards that match it answer. Each round learns at
         * least one bit, so 40 rounds always finish a level. */
        uint8_t resp[7] = {SEL[lvl], 0x20};
        int known = 0;
        for (int round = 0; known < 40; round++) {
            if (round >= 40) {
                return ST25R_ERR_FRAMING;
            }
            rc = anticol_round(d, resp, known);
            if (rc < 0) {
                return rc;
            }
            if (rc < 40) {
                resp[2 + rc / 8] |= (uint8_t)(1u << (rc % 8));
                out->collided = true;
                rc++;
            }
            known = rc;
        }
        uint8_t *cln = resp + 2;
        uint8_t bcc = (uint8_t)(cln[0] ^ cln[1] ^ cln[2] ^ cln[3]);
        if (bcc != cln[4]) {
            return ST25R_ERR_FRAMING;
        }

        /* SELECT: SEL + NVB=0x70 + the 4 UID bytes + BCC, with CRC. The card returns SAK. */
        uint8_t sel[7] = {SEL[lvl], 0x70, cln[0], cln[1], cln[2], cln[3], cln[4]};
        uint8_t sak[4];
        rc = xfer(d, sel, 56, ST25R_TX_CRC, sak, sizeof(sak), NULL, 20);
        if (rc < 1) {
            return rc < 0 ? rc : ST25R_ERR_FRAMING;
        }
        out->sak = sak[0];

        /* Accumulate the UID. A level that carries the cascade tag (CT=0x88) contributes its
         * last three bytes; a final level contributes all four. */
        if (cln[0] == 0x88) {
            if (out->uid_len + 3 <= (int)sizeof(out->uid)) {
                out->uid[out->uid_len++] = cln[1];
                out->uid[out->uid_len++] = cln[2];
                out->uid[out->uid_len++] = cln[3];
            }
        } else {
            if (out->uid_len + 4 <= (int)sizeof(out->uid)) {
                out->uid[out->uid_len++] = cln[0];
                out->uid[out->uid_len++] = cln[1];
                out->uid[out->uid_len++] = cln[2];
                out->uid[out->uid_len++] = cln[3];
            }
        }

        if (!(sak[0] & 0x04)) { /* cascade complete (SAK cascade bit clear) */
            return ST25R_OK;
        }
    }
    return ST25R_ERR_FRAMING; /* ran out of cascade levels without completing */
}

int st25r3916_nfca_halt(st25r_t *d)
{
    /* HLTA: 50 00 + CRC. The card does not answer; any answer within the 1 ms window means
     * it was not acknowledged (ISO 14443-3 §6.4.3). */
    static const uint8_t hlta[2] = {0x50, 0x00};
    uint8_t rx[4];
    int rc = xfer(d, hlta, 16, ST25R_TX_CRC, rx, sizeof(rx), NULL, 2);
    if (rc == ST25R_ERR_TIMEOUT) {
        return ST25R_OK;
    }
    return rc < 0 ? rc : ST25R_ERR_FRAMING;
}

/* ===== RF: ISO 14443-B reader ===========================================================
 * UNTESTED ON HARDWARE. ISO 14443-3 §7. The chip adds SOF/EOF and CRC_B. */

static int nfcb_frame(st25r_t *d, const uint8_t *tx, int n, uint8_t *rx, int cap)
{
    return xfer(d, tx, n * 8, ST25R_TX_CRC, rx, cap, NULL, 10);
}

int st25r3916_nfcb_poll(st25r_t *d, uint8_t afi, bool wupb, st25r_nfcb_t *out, int max)
{
    int found = 0;
    /* Round 0 asks with one slot; if answers collide, ask with 16 (N code 4) and walk the
     * slots with Slot-MARKERs (APn = (n - 1) << 4 | 5), a few times, as ISO 14443-3 §7.4. */
    for (int round = 0; round < 4 && found < max; round++) {
        int nslots = round == 0 ? 1 : 16;
        bool collided = false;
        for (int slot = 1; slot <= nslots && found < max; slot++) {
            uint8_t rx[20];
            int rc;
            if (slot == 1) {
                uint8_t req[3] = {0x05, afi, (uint8_t)((wupb ? 0x08 : 0x00) | (round ? 4 : 0))};
                rc = nfcb_frame(d, req, 3, rx, sizeof(rx));
            } else {
                uint8_t marker = (uint8_t)(((slot - 1) << 4) | 0x05);
                rc = nfcb_frame(d, &marker, 1, rx, sizeof(rx));
            }
            if (rc == ST25R_ERR_TIMEOUT) {
                continue; /* nobody in this slot */
            }
            if (rc == ST25R_ERR_CRC || rc == ST25R_ERR_FRAMING || rc == ST25R_ERR_COLLISION ||
                rc == ST25R_ERR_NOMEM || (rc >= 0 && (rc < 12 || rx[0] != 0x50))) {
                collided = true; /* two cards in one slot garble each other */
                continue;
            }
            if (rc < 0) {
                return rc;
            }
            bool dup = false;
            for (int i = 0; i < found; i++) {
                dup |= !memcmp(out[i].pupi, rx + 1, 4);
            }
            if (!dup) {
                memcpy(out[found].pupi, rx + 1, 4);
                memcpy(out[found].app_data, rx + 5, 4);
                memcpy(out[found].prot_info, rx + 9, 3);
                found++;
            }
        }
        if (!collided) {
            break;
        }
    }
    return found;
}

int st25r3916_nfcb_attrib(st25r_t *d, const st25r_nfcb_t *card)
{
    /* ATTRIB: 1D, PUPI, Param 1 (default TR0/TR1, SOF/EOF), Param 2 (106 kbit/s both ways,
     * FSDI 8 = 256 bytes), Param 3 (the protocol type from the ATQB), Param 4 (CID 0). */
    uint8_t tx[9] = {0x1D, card->pupi[0], card->pupi[1], card->pupi[2], card->pupi[3],
                     0x00, 0x08, (uint8_t)(card->prot_info[1] & 0x0F), 0x00};
    uint8_t rx[8];
    int rc = nfcb_frame(d, tx, 9, rx, sizeof(rx));
    if (rc < 0) {
        return rc;
    }
    return rc < 1 ? ST25R_ERR_FRAMING : rx[0];
}

int st25r3916_nfcb_halt(st25r_t *d, const uint8_t pupi[4])
{
    uint8_t tx[5] = {0x50, pupi[0], pupi[1], pupi[2], pupi[3]};
    uint8_t rx[4];
    int rc = nfcb_frame(d, tx, 5, rx, sizeof(rx));
    if (rc < 0) {
        return rc;
    }
    return (rc >= 1 && rx[0] == 0x00) ? ST25R_OK : ST25R_ERR_FRAMING;
}

/* ===== RF: FeliCa reader ================================================================
 * UNTESTED ON HARDWARE. The chip adds the preamble, sync code and CRC (DS13541 §4.4.4). */

static int nfcf_xfer(st25r_t *d, const uint8_t *tx, int len, uint8_t *rx, int rx_cap,
                     int timeout_ms)
{
    if (!d->nfcf_host_len) {
        return xfer(d, tx, len * 8, ST25R_TX_CRC, rx, rx_cap, NULL, timeout_ms);
    }
    /* The host's LEN byte: LEN counts itself and the payload, not the CRC. */
    uint8_t buf[256];
    if (len < 0 || len > 254) {
        return ST25R_ERR_PARAM;
    }
    buf[0] = (uint8_t)(len + 1);
    memcpy(buf + 1, tx, (size_t)len);
    int n = xfer(d, buf, (len + 1) * 8, ST25R_TX_CRC, buf, sizeof(buf), NULL, timeout_ms);
    if (n < 0) {
        return n;
    }
    if (n < 1 || buf[0] < 1 || buf[0] > n) {
        return ST25R_ERR_FRAMING;
    }
    int plen = buf[0] - 1;
    if (plen > rx_cap) {
        return ST25R_ERR_NOMEM;
    }
    memcpy(rx, buf + 1, (size_t)plen);
    return plen;
}

int st25r3916_nfcf_poll(st25r_t *d, uint16_t sys_code, uint8_t rc, uint8_t tsn,
                        st25r_nfcf_t *out)
{
    if (tsn != 0 && tsn != 1 && tsn != 3 && tsn != 7 && tsn != 15) {
        return ST25R_ERR_PARAM;
    }
    memset(out, 0, sizeof(*out));
    /* SENSF_REQ: 00, system code, request code, time slots. Answers come in slots of
     * 1.2 ms after a 2.4 ms wait; the chip keeps the first clean frame. */
    uint8_t req[5] = {0x00, (uint8_t)(sys_code >> 8), (uint8_t)sys_code, rc, tsn};
    uint8_t rx[24];
    int n = nfcf_xfer(d, req, 5, rx, sizeof(rx), 4 + 2 * (tsn + 1));
    if (n < 0) {
        return n;
    }
    if (n < 17 || rx[0] != 0x01) {
        return ST25R_ERR_FRAMING;
    }
    memcpy(out->idm, rx + 1, 8);
    memcpy(out->pmm, rx + 9, 8);
    if (rc != 0 && n >= 19) {
        out->rd[0] = rx[17];
        out->rd[1] = rx[18];
        out->has_rd = true;
    }
    return ST25R_OK;
}

/* ===== RF: ISO 15693 reader (stream mode, driver-side coding) ===========================
 * UNTESTED ON HARDWARE; see the header for the stream-format assumptions. */

uint16_t st25r_nfcv_crc(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF; /* ISO/IEC 13239: poly x^16+x^12+x^5+1 reflected, init FFFF */
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
        }
    }
    return (uint16_t)~crc;
}

int st25r_nfcv_encode(const uint8_t *data, int len, uint8_t *out, int cap)
{
    /* Each FIFO bit is one 9.44 us slot, LSB first. 1-out-of-4 (ISO 15693-2 §7.2): each
     * pair of bits, least significant pair first, is a 75.52 us symbol of 8 slots with one
     * pulse in slot 2v+1. SOF: pulses in slots 0 and 5; EOF: a pulse in slot 2. */
    if (len < 0 || cap < 4 * len + 2) {
        return ST25R_ERR_NOMEM;
    }
    int n = 0;
    out[n++] = 0x21;
    for (int i = 0; i < len; i++) {
        for (int p = 0; p < 4; p++) {
            int v = (data[i] >> (2 * p)) & 3;
            out[n++] = (uint8_t)(1u << (2 * v + 1));
        }
    }
    out[n++] = 0x04;
    return n;
}

static int sbit(const uint8_t *s, int i)
{
    return (s[i / 8] >> (i % 8)) & 1;
}

/* EOF from the card: logic 0 (sub-carrier, quiet), 24 pulses (3 report bits), quiet. */
static bool nfcv_is_eof(const uint8_t *s, int nbits, int i)
{
    static const uint8_t eof[5] = {1, 0, 1, 1, 1};
    for (int k = 0; k < 5; k++) {
        if (i + k >= nbits || sbit(s, i + k) != eof[k]) {
            return false;
        }
    }
    return i + 5 >= nbits || !sbit(s, i + 5);
}

int st25r_nfcv_decode(const uint8_t *stream, int nbits, uint8_t *out, int cap)
{
    /* One report bit per half data bit; 1 = sub-carrier seen. The receiver starts on the
     * first sub-carrier, so the SOF's leading quiet part is not there: 1 1 1 (24 pulses),
     * then logic 1 (quiet, sub-carrier). Data: logic 0 = 1 0, logic 1 = 0 1, LSB first;
     * 1 1 means two cards answered different bits (a collision). */
    static const uint8_t sof[5] = {1, 1, 1, 0, 1};
    int i = 0;
    while (i < nbits && i < 8 && !sbit(stream, i)) {
        i++; /* tolerate a few quiet report bits before the frame */
    }
    for (int k = 0; k < 5; k++, i++) {
        if (i >= nbits || sbit(stream, i) != sof[k]) {
            return ST25R_ERR_FRAMING;
        }
    }
    int n = 0;
    while (!nfcv_is_eof(stream, nbits, i)) {
        uint8_t byte = 0;
        for (int b = 0; b < 8; b++, i += 2) {
            if (i + 1 >= nbits) {
                return ST25R_ERR_FRAMING; /* ran out before an EOF */
            }
            int h1 = sbit(stream, i), h2 = sbit(stream, i + 1);
            if (h1 && h2) {
                return ST25R_ERR_COLLISION;
            }
            if (!h1 && !h2) {
                return ST25R_ERR_FRAMING;
            }
            if (h2) {
                byte |= (uint8_t)(1u << b);
            }
        }
        if (n >= cap) {
            return ST25R_ERR_NOMEM;
        }
        out[n++] = byte;
    }
    return n;
}

static int nfcv_xfer(st25r_t *d, const uint8_t *req, int req_len, bool crc, uint8_t *resp,
                     int resp_cap, int timeout_ms)
{
    uint8_t frame[ST25R_NFCV_MAX_REQ + 2];
    uint8_t buf[4 * (ST25R_NFCV_MAX_REQ + 2) + 2]; /* the coded request, then the answer */
    if (req_len < 1 || req_len > ST25R_NFCV_MAX_REQ) {
        return ST25R_ERR_PARAM;
    }
    memcpy(frame, req, (size_t)req_len);
    int flen = req_len;
    if (crc) {
        uint16_t c = st25r_nfcv_crc(frame, req_len);
        frame[flen++] = (uint8_t)c;
        frame[flen++] = (uint8_t)(c >> 8);
    }
    int n = st25r_nfcv_encode(frame, flen, buf, sizeof(buf));
    if (n < 0) {
        return n;
    }

    st25r3916_cmd(d, ST25R_CMD_STOP);
    st25r3916_cmd(d, ST25R_CMD_RESET_RXGAIN);
    st25r3916_write_reg(d, ST25R_REG_NUM_TX_BYTES1, ST25R_NTX1(n));
    st25r3916_write_reg(d, ST25R_REG_NUM_TX_BYTES2, ST25R_NTX2(n, 0));
    st25r3916_write_fifo(d, buf, (size_t)n);
    st25r3916_get_interrupts(d);
    st25r3916_cmd(d, ST25R_CMD_TRANSMIT_WITHOUT_CRC);
    /* ~0.6 ms per request byte at 26 kbit/s; allow for it on top of the answer's wait. */
    if (!(wait_irqs(d, IRQ_M(ST25R_IRQ_TXE), 5 + flen) & IRQ_M(ST25R_IRQ_TXE))) {
        return ST25R_ERR_TIMEOUT;
    }
    uint32_t irq = wait_irqs(d, IRQ_M(ST25R_IRQ_RXE) | IRQ_T(ST25R_IRQ_NRE) |
                                    IRQ_E(ST25R_IRQ_ERR1 | ST25R_IRQ_ERR2), timeout_ms);
    if (!(irq & IRQ_M(ST25R_IRQ_RXE))) {
        return (irq & IRQ_E(ST25R_IRQ_ERR1 | ST25R_IRQ_ERR2)) ? ST25R_ERR_FRAMING
                                                               : ST25R_ERR_TIMEOUT;
    }

    uint8_t fs[2];
    st25r3916_read_regs(d, ST25R_REG_FIFO_STATUS1, fs, 2);
    int count = fs[0] | ((fs[1] >> ST25R_FIFO2_B9_B8_SHIFT) << 8);
    int lb = (fs[1] & ST25R_FIFO2_LB_MASK) >> ST25R_FIFO2_LB_SHIFT;
    if (count > (2 * (ST25R_NFCV_MAX_RESP + 2) + 2) || count > (int)sizeof(buf)) {
        return ST25R_ERR_NOMEM;
    }
    if (count > 0) {
        st25r3916_read_fifo(d, buf, (size_t)count);
    }
    int nbits = (lb && count) ? (count - 1) * 8 + lb : count * 8;
    uint8_t dec[ST25R_NFCV_MAX_RESP + 2];
    int m = st25r_nfcv_decode(buf, nbits, dec, sizeof(dec));
    if (m < 0) {
        return m;
    }
    if (crc) {
        if (m < 3) {
            return ST25R_ERR_FRAMING;
        }
        uint16_t c = st25r_nfcv_crc(dec, m - 2);
        if (dec[m - 2] != (uint8_t)c || dec[m - 1] != (uint8_t)(c >> 8)) {
            return ST25R_ERR_CRC;
        }
        m -= 2;
    }
    if (m > resp_cap) {
        return ST25R_ERR_NOMEM;
    }
    memcpy(resp, dec, (size_t)m);
    return m;
}

int st25r3916_nfcv_transceive(st25r_t *d, const uint8_t *req, int req_len, uint8_t *resp,
                              int resp_cap, int timeout_ms)
{
    return nfcv_xfer(d, req, req_len, true, resp, resp_cap, timeout_ms);
}

/* One-slot INVENTORY with a `mlen`-bit mask (the UID's low bits). */
static int nfcv_inventory1(st25r_t *d, int mlen, const uint8_t mask[8], st25r_nfcv_t *card)
{
    uint8_t req[3 + 8] = {0x26, 0x01, (uint8_t)mlen}; /* high rate, inventory, 1 slot */
    int mb = (mlen + 7) / 8;
    for (int i = 0; i < mb; i++) {
        req[3 + i] = mask[i];
    }
    if (mlen % 8) {
        req[3 + mb - 1] &= (uint8_t)((1u << (mlen % 8)) - 1u);
    }
    uint8_t resp[16];
    int rc = nfcv_xfer(d, req, 3 + mb, true, resp, sizeof(resp), 20);
    if (rc == ST25R_ERR_CRC) {
        return ST25R_ERR_COLLISION; /* answers that overlap badly fail the CRC */
    }
    if (rc < 0) {
        return rc;
    }
    if (rc < 10 || (resp[0] & 0x01)) {
        return ST25R_ERR_FRAMING;
    }
    card->dsfid = resp[1];
    memcpy(card->uid, resp + 2, 8);
    return ST25R_OK;
}

int st25r3916_nfcv_inventory(st25r_t *d, st25r_nfcv_t *out, int max)
{
    /* Depth-first search over the UID's bits: a collision with an n-bit mask means two or
     * more cards share those bits, so ask with n+1 bits (0 first, then 1). Every bit in the
     * mask was added at a collision, so backtracking flips the deepest 0 to 1. */
    uint8_t mask[8] = {0};
    int len = 0;
    int found = 0;
    for (int guard = 0; guard < 4 * 64 + 8 && found < max; guard++) {
        st25r_nfcv_t c;
        int rc = nfcv_inventory1(d, len, mask, &c);
        if (rc == ST25R_ERR_COLLISION && len < 64) {
            mask[len / 8] &= (uint8_t)~(1u << (len % 8));
            len++;
            continue;
        }
        if (rc == ST25R_OK) {
            bool dup = false;
            for (int i = 0; i < found; i++) {
                dup |= !memcmp(out[i].uid, c.uid, 8);
            }
            if (!dup) {
                out[found++] = c;
            }
        } else if (rc != ST25R_ERR_TIMEOUT && rc != ST25R_ERR_COLLISION &&
                   rc != ST25R_ERR_FRAMING) {
            return rc;
        }
        while (len > 0 && (mask[(len - 1) / 8] & (1u << ((len - 1) % 8)))) {
            mask[(len - 1) / 8] &= (uint8_t)~(1u << ((len - 1) % 8));
            len--;
        }
        if (len == 0) {
            break;
        }
        mask[(len - 1) / 8] |= (uint8_t)(1u << ((len - 1) % 8));
    }
    return found;
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
