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
    /* Optional: block until the chip's IRQ pin is active or `timeout_ms` elapses; return
     * true if it went active. If NULL, the RF flows poll the interrupt registers instead
     * (works, but busier). Wire this to a semaphore your IRQ-pin ISR gives. */
    bool (*wait_irq)(void *ctx, int timeout_ms);
} st25r_bus_t;

/* ---- Analog configuration table --------------------------------------------------------
 *
 * The receiver, modulation and regulator settings that make the RF actually work depend on
 * your antenna and matching network, so the driver does not hardcode them. Instead a board
 * supplies a table of register tweaks to apply at each RF event; the driver's reset defaults
 * are used for anything the table leaves alone. Derive the values from the datasheet's
 * receiver/modulation registers and tune them against your antenna (ST's reference-antenna
 * numbers from RFAL are a starting point, but they are ST's and are not reproduced here).
 */
typedef enum {
    ST25R_ANALOG_POWER_UP = 0, /* oscillator ready: chip-level analog (regulators, Rx) */
    ST25R_ANALOG_POLL_A,       /* NFC-A reader: Rx gain/threshold, Tx modulation */
    ST25R_ANALOG_LISTEN_A,     /* NFC-A card emulation */
    ST25R_ANALOG_WAKEUP,       /* wake-up measurement tuning */
    ST25R_ANALOG_POLL_B,       /* ISO 14443-B reader (AM modulation index, Rx) */
    ST25R_ANALOG_POLL_F,       /* FeliCa reader */
    ST25R_ANALOG_POLL_V,       /* ISO 15693 reader (stream mode) */
    ST25R_ANALOG_EVENT_COUNT,
} st25r_analog_event_t;

/* One register tweak: at `event`, reg = (reg & ~mask) | (value & mask), in space A, or space
 * B when `spaceb` is set. */
typedef struct {
    uint8_t event;  /* an st25r_analog_event_t */
    uint8_t reg;
    uint8_t mask;
    uint8_t value;
    uint8_t spaceb; /* 0: register space A, 1: space B */
} st25r_analog_entry_t;

typedef struct {
    const st25r_analog_entry_t *entries;
    size_t count;
} st25r_analog_config_t;

typedef struct {
    const st25r_bus_t *bus;
    uint8_t ic_type; /* filled by st25r3916_probe() */
    uint8_t ic_rev;
    const st25r_analog_config_t *analog; /* optional; NULL = chip reset defaults only */
    /* Reader settings; zero is the default for each. */
    uint8_t tech;     /* the current st25r_tech_t (set by field_on / set_tech) */
    bool nfcf_host_len; /* FeliCa: the host writes and reads the LEN byte (see below) */
    bool nfcv_am;       /* ISO 15693: 10 % AM (the analog table sets the index), not OOK */
} st25r_t;

/* Negative return codes for the RF flows below. */
enum {
    ST25R_OK = 0,
    ST25R_ERR_NODEV = -1,     /* probe failed: no ST25R3916B on the bus */
    ST25R_ERR_TIMEOUT = -2,   /* expected interrupt / response did not arrive */
    ST25R_ERR_COLLISION = -3, /* bit collision in a frame, or another reader's field */
    ST25R_ERR_CRC = -4,       /* CRC or parity error in the response */
    ST25R_ERR_FRAMING = -5,   /* framing error / malformed response */
    ST25R_ERR_NOMEM = -6,     /* caller's buffer too small */
    ST25R_ERR_PARAM = -7,     /* bad argument */
};

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

/* ---- Analog configuration -------------------------------------------------------------- */
/* Install the board's analog-config table (held by reference; NULL clears it). The RF flows
 * below apply the entries for each event automatically; with no table they run on the chip's
 * reset defaults. */
void st25r3916_set_analog_config(st25r_t *d, const st25r_analog_config_t *cfg);
/* Apply every table entry registered for `event`. Called by the RF flows; also exposed so a
 * board can re-apply a set at its own points. No-op if no table is installed. */
void st25r3916_apply_analog_config(st25r_t *d, st25r_analog_event_t event);

/* ---- RF: power-up and ISO 14443-A (NFC-A) reader ---------------------------------------
 *
 * UNTESTED ON HARDWARE. These follow DS13541 §4.2.13 ("Reader operation") and the ISO
 * 14443-3 anticollision procedure. They assume a 4-wire SPI with the IRQ pin wired (see
 * st25r_bus_t.wait_irq). Timings (guard time, timeouts) are conservative starting points to
 * confirm against the datasheet and a real card.
 */

/* Bring the chip to Ready (oscillator + regulators), set NFC-A reader mode at 106 kbit/s,
 * adjust the regulators, enable Rx/Tx, and switch the RF field on with the ISO 14443 guard
 * time. Call st25r3916_probe() first. Returns ST25R_OK or a negative ST25R_ERR_*. Analog
 * settings (receiver/modulation registers) keep their reset defaults here; a real design
 * loads the values its antenna needs first. */
int st25r3916_field_on(st25r_t *d);

/* Reader protocols. */
typedef enum {
    ST25R_TECH_A = 0, /* ISO 14443-A / NFC-A */
    ST25R_TECH_B,     /* ISO 14443-B / NFC-B */
    ST25R_TECH_F,     /* FeliCa / NFC-F */
    ST25R_TECH_V,     /* ISO 15693 / NFC-V (26 kbit/s, coded by the driver in stream mode) */
} st25r_tech_t;

/* As st25r3916_field_on(), for any protocol: `br` is the ST25R_BR_* bit rate both ways
 * (A and B: 106..848; F: 212 or 424; V: ignored). */
int st25r3916_field_on_tech(st25r_t *d, st25r_tech_t tech, uint8_t br);

/* Switch the protocol and bit rates with the field on, e.g. to 424 kbit/s after an
 * ISO 14443-4 PPS (A) or ATTRIB (B), which the caller sends. Applies that protocol's
 * analog-config event. Returns ST25R_OK or ST25R_ERR_PARAM for a rate the protocol lacks. */
int st25r3916_set_tech(st25r_t *d, st25r_tech_t tech, uint8_t br_tx, uint8_t br_rx);

/* Switch the RF field off and return the chip to a low-power idle (oscillator off). */
void st25r3916_field_off(st25r_t *d);

/* Flags for st25r3916_transceive(). */
enum {
    ST25R_TX_CRC = 1u << 0,      /* append (and expect) the ISO 14443 CRC */
    ST25R_TX_ANTICOL = 1u << 1,  /* this is a bit-oriented anticollision frame (sets antcl) */
    ST25R_TX_NO_PARITY = 1u << 2, /* raw: no parity generated/checked (e.g. MIFARE layers) */
};

/* Send `tx_bits` bits from `tx` and receive the card's answer into `rx` (capacity
 * `rx_cap` bytes). `*rx_bits` (may be NULL) receives the answer's length in bits (the last
 * byte may be partial). Returns the answer length in bytes (rounded up) or a negative
 * ST25R_ERR_*. The field must already be on.
 *
 * In the current protocol (d->tech): A and B as given (the chip adds and checks the CRC with
 * ST25R_TX_CRC; the answer keeps its 2 CRC bytes). F: whole bytes, without the LEN byte
 * either way, and always with CRC. V: whole bytes; the driver codes the frame, adds the CRC
 * with ST25R_TX_CRC and checks and strips it from the answer (st25r3916_nfcv_transceive).
 *
 * FeliCa LEN byte: DS13541 §4.4.4 says Transmit without CRC "skips the Length and CRC
 * bytes", so the driver assumes the chip adds LEN on transmit and takes it off on receive.
 * If a real chip shows otherwise, set d->nfcf_host_len and the driver handles LEN itself. */
int st25r3916_transceive(st25r_t *d, const uint8_t *tx, int tx_bits, unsigned flags,
                         uint8_t *rx, int rx_cap, int *rx_bits, int timeout_ms);

typedef struct {
    uint8_t atqa[2];  /* answer to request (SENS_RES) */
    uint8_t uid[10];  /* 4, 7 or 10 bytes */
    uint8_t uid_len;
    uint8_t sak;      /* select acknowledge (SAK) of the last cascade level */
    bool collided;    /* other cards answered too: HLTA this one, then select again */
} st25r_nfca_t;

/* ISO 14443-A REQA (0x26) short frame; returns the 2-byte ATQA or ST25R_ERR_TIMEOUT if no
 * card answered. Use WUPA (true) to also wake halted cards. */
int st25r3916_nfca_request(st25r_t *d, uint8_t atqa[2], bool wupa);

/* Full ISO 14443-A activation: REQA, then the anticollision + SELECT cascade (levels 1..3)
 * into `out` (UID, ATQA, SAK). With several cards in the field the bit collisions are
 * resolved (ISO 14443-3 §6.5.3, choosing 1 at each collided bit), so exactly one card ends
 * up selected and `out->collided` is set. To find the others, HLTA the selected card with
 * st25r3916_nfca_halt() and call this again; ST25R_ERR_TIMEOUT means none are left. Halted
 * cards stay quiet until the field drops or a WUPA. Returns ST25R_OK or a negative
 * ST25R_ERR_*. */
int st25r3916_nfca_select(st25r_t *d, st25r_nfca_t *out);

/* Send HLTA to the selected card (it goes to HALT and ignores REQA). Returns ST25R_OK. */
int st25r3916_nfca_halt(st25r_t *d);

/* ---- RF: ISO 14443-B (NFC-B) reader --------------------------------------------------
 *
 * UNTESTED ON HARDWARE. ISO 14443-3 type B: REQB/WUPB with time slots, ATTRIB, HLTB. Start
 * with st25r3916_field_on_tech(d, ST25R_TECH_B, ST25R_BR_106).
 */
typedef struct {
    uint8_t pupi[4];      /* pseudo-unique PICC identifier */
    uint8_t app_data[4];  /* application data (AFI, CRC_B(AID), applications) */
    uint8_t prot_info[3]; /* bit rates, max frame size, protocol type, FWI, ... */
} st25r_nfcb_t;

/* Find up to `max` type B cards with application family `afi` (0 = all). Tries one slot,
 * and on a collision 16 slots with Slot-MARKERs, for a few rounds. `wupb` also wakes
 * halted cards. Returns the number found (0: none) or a negative ST25R_ERR_*. Found cards
 * wait in READY for ATTRIB; HLTB the ones you are done with. */
int st25r3916_nfcb_poll(st25r_t *d, uint8_t afi, bool wupb, st25r_nfcb_t *out, int max);

/* ATTRIB (select for ISO 14443-4): CID 0, default timings, our max frame 256 bytes at
 * 106 kbit/s, the card's protocol type. Returns the answer's first byte (MBLI/CID, >= 0) or
 * a negative ST25R_ERR_*. */
int st25r3916_nfcb_attrib(st25r_t *d, const st25r_nfcb_t *card);

/* HLTB: put the card to HALT. Returns ST25R_OK or a negative ST25R_ERR_*. */
int st25r3916_nfcb_halt(st25r_t *d, const uint8_t pupi[4]);

/* ---- RF: FeliCa (NFC-F) reader -------------------------------------------------------
 *
 * UNTESTED ON HARDWARE. JIS X 6319-4 polling (SENSF_REQ). Start with
 * st25r3916_field_on_tech(d, ST25R_TECH_F, ST25R_BR_212) (or 424).
 */
typedef struct {
    uint8_t idm[8]; /* manufacture ID (NFCID2) */
    uint8_t pmm[8]; /* manufacture parameter */
    uint8_t rd[2];  /* request data: the system code when asked for (has_rd) */
    bool has_rd;
} st25r_nfcf_t;

/* Poll for a card with system code `sys_code` (0xFFFF = any). `rc` is the request code
 * (0: none, 1: return the system code, 2: communication performance). `tsn` is the number
 * of time slots minus one (0, 1, 3, 7 or 15). Returns ST25R_OK with the first card that
 * answered cleanly, ST25R_ERR_TIMEOUT if none, ST25R_ERR_CRC if answers collided (poll
 * again with more slots), or another negative ST25R_ERR_*. */
int st25r3916_nfcf_poll(st25r_t *d, uint16_t sys_code, uint8_t rc, uint8_t tsn,
                        st25r_nfcf_t *out);

/* ---- RF: ISO 15693 (NFC-V) reader -----------------------------------------------------
 *
 * UNTESTED ON HARDWARE. The chip has no ISO 15693 framing: it sends and receives in stream
 * mode (DS13541 Table 24/34) and the driver codes the frames itself: 1-out-of-4 pulse
 * position at 26 kbit/s out, single-subcarrier Manchester in, both with SOF/EOF and the
 * ISO/IEC 13239 CRC. DS13541 does not describe the stream format, so these are assumptions
 * to check on a chip: FIFO bits go out and come in LSB first, one bit per stx period
 * (fc/128) out and per report period (8 pulses of the 424 kHz subcarrier, half a data bit)
 * in, 1 = a pulse / subcarrier seen. Start with st25r3916_field_on_tech(d, ST25R_TECH_V, 0).
 */
#define ST25R_NFCV_MAX_REQ 64  /* longest request (without CRC) the driver codes */
#define ST25R_NFCV_MAX_RESP 72 /* longest answer (without CRC) it decodes */

typedef struct {
    uint8_t uid[8]; /* as sent, LSB first (uid[7] is 0xE0) */
    uint8_t dsfid;
} st25r_nfcv_t;

/* Send request `req` (flags, command, parameters; no CRC) and receive the answer (flags
 * byte first, CRC checked and removed) into `resp`. Returns the answer length, or
 * ST25R_ERR_TIMEOUT, ST25R_ERR_COLLISION (two cards answered), ST25R_ERR_CRC,
 * ST25R_ERR_FRAMING, ST25R_ERR_NOMEM, ST25R_ERR_PARAM. */
int st25r3916_nfcv_transceive(st25r_t *d, const uint8_t *req, int req_len, uint8_t *resp,
                              int resp_cap, int timeout_ms);

/* Find up to `max` cards: one-slot INVENTORY, and on a collision a search that grows the
 * UID mask one bit at a time (ISO 15693-3 §8 allows any mask). Returns the number found or
 * a negative ST25R_ERR_*. */
int st25r3916_nfcv_inventory(st25r_t *d, st25r_nfcv_t *out, int max);

/* The ISO 15693 coding, exposed for tests and for other front ends. */
uint16_t st25r_nfcv_crc(const uint8_t *data, int len); /* ISO/IEC 13239; send LSB first */
/* Code `len` bytes (CRC included) as SOF + 1-out-of-4 + EOF stream bits. Returns bytes
 * written to `out` (4 * len + 2), or ST25R_ERR_NOMEM. */
int st25r_nfcv_encode(const uint8_t *data, int len, uint8_t *out, int cap);
/* Decode a received stream (`nbits` bits) into bytes (CRC included). Returns the byte
 * count, or ST25R_ERR_COLLISION / ST25R_ERR_FRAMING / ST25R_ERR_NOMEM. */
int st25r_nfcv_decode(const uint8_t *stream, int nbits, uint8_t *out, int cap);

/* ---- RF: ISO 14443-A card emulation (listen / target mode) -----------------------------
 *
 * UNTESTED ON HARDWARE. The chip answers REQA, anticollision and SELECT automatically from
 * PT_Memory (DS13541 §4.2.14, Table 8/33); the host loads the identity, then waits for the
 * reader to select the card and handles the higher-layer frames (RATS/APDU) itself.
 */

/* The identity this device presents as an NFC-A tag. */
typedef struct {
    uint8_t nfcid1[7]; /* UID; the reader-visible NFCID1 */
    uint8_t nfcid1_len; /* 4 or 7 */
    uint8_t atqa[2];    /* SENS_RES answered to REQA/WUPA */
    uint8_t sak;        /* SEL_RES (SAK) of the final cascade level */
} st25r_nfca_target_t;

typedef enum {
    ST25R_LISTEN_NONE = 0,   /* nothing happened before the timeout */
    ST25R_LISTEN_ACTIVATED,  /* a reader completed anticollision and selected this card */
    ST25R_LISTEN_RECEIVED,   /* a reader frame is in `rx` (rx_len bytes) and needs a reply */
    ST25R_LISTEN_FIELD_OFF,  /* the reader's field went away */
} st25r_listen_event_t;

/* Enter NFC-A card emulation: load PT_Memory with `t`, configure passive-target mode with
 * automatic anticollision, enable the external field detector and go to the Sense state to
 * wait for a reader. Returns ST25R_OK or a negative ST25R_ERR_*. */
int st25r3916_listen_start(st25r_t *d, const st25r_nfca_target_t *t);

/* Leave listen mode and return the chip to idle. */
void st25r3916_listen_stop(st25r_t *d);

/* Wait up to `timeout_ms` for a listen-mode event. On ST25R_LISTEN_RECEIVED the reader's
 * frame is copied into `rx` (capacity `rx_cap`) and its length put in `*rx_len` (may be
 * NULL). Reply to it with st25r3916_listen_respond(). */
st25r_listen_event_t st25r3916_listen_poll(st25r_t *d, uint8_t *rx, int rx_cap, int *rx_len,
                                           int timeout_ms);

/* Send a higher-layer response frame (e.g. ATS, an APDU reply) back to the reader while in
 * listen mode. `with_crc` appends the ISO 14443 CRC. Returns ST25R_OK or a negative error. */
int st25r3916_listen_respond(st25r_t *d, const uint8_t *tx, int tx_len, bool with_crc,
                             int timeout_ms);

/* ---- RF: wake-up mode (low-power tag detection, DS13541 §4.5.67-69) ---------------------
 *
 * UNTESTED ON HARDWARE. The chip sleeps with the main oscillator off and wakes on its own
 * timer to measure the antenna's amplitude and/or phase; when the change from a reference
 * exceeds a threshold (a tag entering the field detunes the antenna) it raises an interrupt
 * on the IRQ pin to wake the host. The host then runs a normal poll to see if a tag is
 * really there. The right thresholds depend on the antenna and must be tuned on hardware.
 */

typedef struct {
    bool measure_amplitude; /* wake on an amplitude change */
    bool measure_phase;     /* wake on a phase change */
    uint8_t period;         /* 0..7: wake-up period = (period + 1) x step */
    bool fast_period;       /* false: 100 ms step, true: 10 ms step */
    uint8_t am_delta;       /* 0..15: amplitude change from the reference that wakes (Δam) */
    uint8_t ph_delta;       /* 0..15: phase change from the reference that wakes (Δpm) */
    bool auto_average;      /* true: track a slow-moving reference (ignores drift); false:
                             * take one reference measurement now and hold it */
} st25r_wakeup_cfg_t;

/* Flags returned by st25r3916_wakeup_cause(). */
enum {
    ST25R_WAKE_TIMER = 1u << 0,     /* the wake-up timer elapsed (if the timer IRQ is on) */
    ST25R_WAKE_AMPLITUDE = 1u << 1, /* the amplitude changed past Δam */
    ST25R_WAKE_PHASE = 1u << 2,     /* the phase changed past Δpm */
};

/* Enter wake-up mode per `cfg`: configure the measurement(s), establish the reference,
 * mask every interrupt except the wake sources, and power down to periodic sensing. The
 * host should then sleep until the IRQ pin wakes it. Returns ST25R_OK or a negative error. */
int st25r3916_wakeup_start(st25r_t *d, const st25r_wakeup_cfg_t *cfg);

/* After the IRQ wakes the host, read why: a bitmask of ST25R_WAKE_*. */
int st25r3916_wakeup_cause(st25r_t *d);

/* Leave wake-up mode and restore the default (unmasked) interrupts. */
void st25r3916_wakeup_stop(st25r_t *d);

#endif
