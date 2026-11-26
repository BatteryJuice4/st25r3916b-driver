/*
 * st25r3916_regs.h — ST25R3916B register map, direct commands and SPI framing.
 *
 * Written from the ST datasheet DS13541 Rev 8 (ST25R3916B / ST25R3917B / ST25R3919B).
 * These are facts about the chip (addresses, command codes, bit positions), transcribed
 * from the datasheet, not from ST's RFAL middleware. Section/table references below point
 * into DS13541 so every value can be re-checked against it.
 *
 * SPI (DS13541 §4.3.3): clock polarity 0, clock phase 1 (SPI mode 1), MSB first, active-low
 * select. The first byte after select-low picks the operation (Table 12): the top two bits
 * are the mode, the low six carry a register address or an operation selector.
 */
#ifndef PDA_DRV_ST25R3916_REGS_H
#define PDA_DRV_ST25R3916_REGS_H

/* ---- SPI operation-mode bytes (DS13541 Table 12) --------------------------------------- */
#define ST25R_SPI_REG_WRITE(a) ((uint8_t)(0x00u | ((a) & 0x3Fu))) /* 00 + addr */
#define ST25R_SPI_REG_READ(a)  ((uint8_t)(0x40u | ((a) & 0x3Fu))) /* 01 + addr */
#define ST25R_SPI_FIFO_LOAD    0x80u                              /* 10 000000 */
#define ST25R_SPI_FIFO_READ    0x9Fu                              /* 10 011111 */
#define ST25R_SPI_PT_LOAD_A    0xA0u                              /* 10 100000 */
#define ST25R_SPI_PT_LOAD_F    0xA8u                              /* 10 101000 */
#define ST25R_SPI_PT_LOAD_TSN  0xACu                              /* 10 101100 */
#define ST25R_SPI_PT_READ      0xBFu                              /* 10 111111 */
/* Direct-command bytes already carry the 11 prefix (Table 14), sent as-is. */
#define ST25R_SPI_SPACEB_PREFIX 0xFBu /* prefix a reg read/write to reach register space B */

/* ---- Direct commands (DS13541 §4.4, Table 14) ------------------------------------------ */
enum {
    ST25R_CMD_SET_DEFAULT = 0xC1,       /* power-up state: stop, reset regs, clear collisions */
    ST25R_CMD_STOP = 0xC2,              /* stop all activities (also clears FIFO) */
    ST25R_CMD_TRANSMIT_WITH_CRC = 0xC4, /* TX, chip appends CRC */
    ST25R_CMD_TRANSMIT_WITHOUT_CRC = 0xC5,
    ST25R_CMD_TRANSMIT_REQA = 0xC6, /* ISO14443A REQA (7-bit short frame) */
    ST25R_CMD_TRANSMIT_WUPA = 0xC7, /* ISO14443A WUPA */
    ST25R_CMD_INITIAL_FIELD_ON = 0xC8,  /* reader: collision avoidance + field on */
    ST25R_CMD_RESPONSE_FIELD_ON = 0xC9, /* target/active: response collision avoidance */
    ST25R_CMD_GOTO_SENSE = 0xCD,        /* passive target -> Sense (Idle) */
    ST25R_CMD_GOTO_SLEEP = 0xCE,        /* passive target -> Sleep (Halt) */
    ST25R_CMD_MASK_RECEIVE_DATA = 0xD0,
    ST25R_CMD_UNMASK_RECEIVE_DATA = 0xD1,
    ST25R_CMD_AM_MOD_STATE = 0xD2,
    ST25R_CMD_MEASURE_AMPLITUDE = 0xD3,
    ST25R_CMD_RESET_RXGAIN = 0xD5,
    ST25R_CMD_ADJUST_REGULATORS = 0xD6, /* set regulated voltages from the supply level */
    ST25R_CMD_CALIBRATE_DRIVER_TIMING = 0xD8,
    ST25R_CMD_MEASURE_PHASE = 0xD9,
    ST25R_CMD_CLEAR_RSSI = 0xDA,
    ST25R_CMD_CLEAR_FIFO = 0xDB,
    ST25R_CMD_TRANSPARENT_MODE = 0xDC,
    ST25R_CMD_MEASURE_POWER_SUPPLY = 0xDF,
    ST25R_CMD_START_GP_TIMER = 0xE0,
    ST25R_CMD_START_WAKEUP_TIMER = 0xE1,
    ST25R_CMD_START_MASK_RECEIVE_TIMER = 0xE2,
    ST25R_CMD_START_NO_RESPONSE_TIMER = 0xE3,
    ST25R_CMD_START_PPON2_TIMER = 0xE4,
    ST25R_CMD_STOP_NO_RESPONSE_TIMER = 0xE8,
    ST25R_CMD_TRIGGER_RC_CALIBRATION = 0xEA,
    ST25R_CMD_SPACE_B_ACCESS = 0xFB,
    ST25R_CMD_TEST_ACCESS = 0xFC,
};

/* ---- Register space A addresses (DS13541 Table 18) ------------------------------------- */
enum {
    ST25R_REG_IO_CONF1 = 0x00,
    ST25R_REG_IO_CONF2 = 0x01,
    ST25R_REG_OP_CONTROL = 0x02,
    ST25R_REG_MODE = 0x03,
    ST25R_REG_BIT_RATE = 0x04,
    ST25R_REG_ISO14443A_NFC = 0x05,
    ST25R_REG_ISO14443B_1 = 0x06,
    ST25R_REG_ISO14443B_FELICA = 0x07,
    ST25R_REG_PASSIVE_TARGET = 0x08,
    ST25R_REG_STREAM_MODE = 0x09,
    ST25R_REG_AUX = 0x0A,
    ST25R_REG_RX_CONF1 = 0x0B,
    ST25R_REG_RX_CONF2 = 0x0C,
    ST25R_REG_RX_CONF3 = 0x0D,
    ST25R_REG_RX_CONF4 = 0x0E,
    ST25R_REG_MASK_RX_TIMER = 0x0F,
    ST25R_REG_NO_RESPONSE_TIMER1 = 0x10,
    ST25R_REG_NO_RESPONSE_TIMER2 = 0x11,
    ST25R_REG_TIMER_EMV_CONTROL = 0x12,
    ST25R_REG_GPT1 = 0x13,
    ST25R_REG_GPT2 = 0x14,
    ST25R_REG_PPON2 = 0x15,
    ST25R_REG_MASK_MAIN_IRQ = 0x16,
    ST25R_REG_MASK_TIMER_NFC_IRQ = 0x17,
    ST25R_REG_MASK_ERROR_WUP_IRQ = 0x18,
    ST25R_REG_MASK_PASSIVE_TARGET_IRQ = 0x19,
    ST25R_REG_MAIN_IRQ = 0x1A,
    ST25R_REG_TIMER_NFC_IRQ = 0x1B,
    ST25R_REG_ERROR_WUP_IRQ = 0x1C,
    ST25R_REG_PASSIVE_TARGET_IRQ = 0x1D,
    ST25R_REG_FIFO_STATUS1 = 0x1E,
    ST25R_REG_FIFO_STATUS2 = 0x1F,
    ST25R_REG_COLLISION = 0x20,
    ST25R_REG_PASSIVE_TARGET_STATUS = 0x21,
    ST25R_REG_NUM_TX_BYTES1 = 0x22,
    ST25R_REG_NUM_TX_BYTES2 = 0x23,
    ST25R_REG_BIT_RATE_DETECT = 0x24,
    ST25R_REG_AD_RESULT = 0x25,
    ST25R_REG_ANT_TUNE1 = 0x26,
    ST25R_REG_ANT_TUNE2 = 0x27,
    ST25R_REG_TX_DRIVER = 0x28,
    ST25R_REG_PT_MOD = 0x29,
    ST25R_REG_FIELD_ON_THRESHOLD = 0x2A,  /* external field detector activation threshold */
    ST25R_REG_FIELD_OFF_THRESHOLD = 0x2B, /* external field detector deactivation threshold */
    ST25R_REG_REGULATOR_CONTROL = 0x2C,
    ST25R_REG_RSSI = 0x2D,
    ST25R_REG_GAIN_REDUCTION = 0x2E,
    ST25R_REG_AUX_DISPLAY = 0x31,
    ST25R_REG_WAKEUP_TIMER_CONTROL = 0x32,
    ST25R_REG_AMPLITUDE_MEAS_CONF = 0x33,
    ST25R_REG_AMPLITUDE_MEAS_REF = 0x34,
    ST25R_REG_AMPLITUDE_MEAS_AA_DISPLAY = 0x35,
    ST25R_REG_AMPLITUDE_MEAS_DISPLAY = 0x36,
    ST25R_REG_PHASE_MEAS_CONF = 0x37,
    ST25R_REG_PHASE_MEAS_REF = 0x38,
    ST25R_REG_PHASE_MEAS_AA_DISPLAY = 0x39,
    ST25R_REG_PHASE_MEAS_DISPLAY = 0x3A,
    ST25R_REG_IC_IDENTITY = 0x3F,
};

/* ---- Register space B addresses (DS13541 Table 19; reach them via the 0xFB prefix) ------ */
enum {
    ST25R_REGB_EMD_SUPPRESSION = 0x05,
    ST25R_REGB_SUBC_START_TIMER = 0x06,
    ST25R_REGB_P2P_RX_CONF = 0x0B,
    ST25R_REGB_CORR_CONF1 = 0x0C,
    ST25R_REGB_CORR_CONF2 = 0x0D,
    ST25R_REGB_SQUELCH_TIMER = 0x0F,
    ST25R_REGB_FIELD_ON_GT = 0x15,
    ST25R_REGB_AUX_MOD = 0x28,
    ST25R_REGB_TX_DRIVER_TIMING = 0x29,
    ST25R_REGB_RES_AM_MOD = 0x2A,
    ST25R_REGB_TX_DRIVER_STATUS = 0x2B,
    ST25R_REGB_REGULATOR_DISPLAY = 0x2C,
    ST25R_REGB_OVERSHOOT_CONF1 = 0x30,
    ST25R_REGB_OVERSHOOT_CONF2 = 0x31,
    ST25R_REGB_UNDERSHOOT_CONF1 = 0x32,
    ST25R_REGB_UNDERSHOOT_CONF2 = 0x33,
};

/* ---- IO configuration register 1 (0x00, DS13541 Table 20) ------------------------------ */
#define ST25R_IO_CONF1_SINGLE (1u << 7) /* single-ended (one RFO) vs differential driving */
#define ST25R_IO_CONF1_RFO2 (1u << 6)
#define ST25R_IO_CONF1_OUT_CL_SHIFT 1 /* MCU_CLK output frequency select (out_cl1:0) */
#define ST25R_IO_CONF1_LF_CLK_OFF (1u << 0)

/* ---- Operation control register (0x02, DS13541 Table 22) ------------------------------- */
#define ST25R_OP_EN (1u << 7)       /* enable oscillator + regulator (Ready) */
#define ST25R_OP_RX_EN (1u << 6)    /* enable receivers */
#define ST25R_OP_RX_CHN (1u << 5)   /* one Rx channel only */
#define ST25R_OP_RX_MAN (1u << 4)   /* manual Rx channel select */
#define ST25R_OP_TX_EN (1u << 3)    /* enable transmitter (field) */
#define ST25R_OP_WU (1u << 2)       /* enable wake-up mode */
#define ST25R_OP_EN_FD_C1 (1u << 1) /* external field detector config, high bit */
#define ST25R_OP_EN_FD_C0 (1u << 0)
#define ST25R_OP_EN_FD_MASK (0x3u << 0) /* en_fd_c1:0 */
#define ST25R_OP_EN_FD_AUTO (0x3u << 0) /* 11: automatic field detector (passive target/AP2P) */

/* ---- Mode definition register (0x03, DS13541 Table 23/24/25) --------------------------- */
#define ST25R_MODE_TARG (1u << 7)  /* 0 initiator, 1 target */
#define ST25R_MODE_OM_SHIFT 3      /* om3:0 operation mode */
#define ST25R_MODE_OM_MASK (0xFu << 3)
#define ST25R_MODE_TR_AM (1u << 2) /* 0 OOK, 1 AM modulation */
#define ST25R_MODE_NFC_AR_SHIFT 0  /* automatic response RF collision avoidance */
/* Initiator om values (Table 24) */
#define ST25R_OM_NFCIP1_ACTIVE 0x0
#define ST25R_OM_ISO14443A 0x1
#define ST25R_OM_ISO14443B 0x2
#define ST25R_OM_FELICA 0x3
#define ST25R_OM_TOPAZ 0x4
#define ST25R_OM_SUBC_STREAM 0xE /* sub-carrier stream mode (ISO 15693 reader) */
#define ST25R_OM_BPSK_STREAM 0xF
/* Target om values (Table 25): ISO14443A passive target = 0x1, FeliCa passive = 0x4 */

/* ---- Bit rate definition register (0x04, DS13541 Table 26/27) -------------------------- */
#define ST25R_BR_TX_SHIFT 4 /* tx_rate1:0 */
#define ST25R_BR_RX_SHIFT 0 /* rx_rate1:0 */
#define ST25R_BR_106 0x0    /* fc/128 ~106 kbit/s */
#define ST25R_BR_212 0x1    /* fc/64  ~212 */
#define ST25R_BR_424 0x2    /* fc/32  ~424 */
#define ST25R_BR_848 0x3    /* fc/16  ~848 */

/* ---- ISO14443A and NFC 106kb/s settings register (0x05, DS13541 Table 28) -------------- */
#define ST25R_A_NO_TX_PAR (1u << 7) /* no parity generated during Tx (raw) */
#define ST25R_A_NO_RX_PAR (1u << 6) /* received parity put in FIFO; no parity/CRC check (raw) */
#define ST25R_A_NFC_F0 (1u << 5)    /* NFCIP-1 transport frame format */
#define ST25R_A_P_LEN_SHIFT 1       /* modulation pulse width (p_len3:0) */
#define ST25R_A_ANTCL (1u << 0)     /* 1 = bit-oriented ISO14443 anticollision frame */

/* ---- ISO14443B settings (0x06/0x07, DS13541 Table 30/31) ------------------------------ *
 * 0x06 = 0: EGT 0, SOF 10 etu low + 2 etu high, EOF 10 etu (ISO 14443-3 minimums).
 * 0x07 = 0: minimum TR1 80/fs, SOF and EOF expected from the PICC, FeliCa preamble 48 bits. */
#define ST25R_B_FELICA_FP_SHIFT 0 /* f_p1:0 FeliCa preamble length 48/64/80/96 */

/* ---- Stream mode definition register (0x09, DS13541 Table 34/35/36) ------------------- */
#define ST25R_STREAM_SCF_SHIFT 5 /* scf1:0 sub-carrier frequency */
#define ST25R_STREAM_SCF_424 0x1 /* fc/32 (424 kHz) */
#define ST25R_STREAM_SCP_SHIFT 3 /* scp1:0 sub-carrier pulses per report period */
#define ST25R_STREAM_SCP_4 0x2
#define ST25R_STREAM_SCP_8 0x3
#define ST25R_STREAM_STX_SHIFT 0 /* stx2:0 Tx modulator period */
#define ST25R_STREAM_STX_128 0x0 /* fc/128 (9.44 us, one ISO 15693 pulse slot) */

/* ---- Auxiliary definition register (0x0A, DS13541 Table 37) ---------------------------- */
#define ST25R_AUXDEF_NO_CRC_RX (1u << 7) /* receive without CRC check */

/* ---- Number of transmitted bytes registers (0x22/0x23, DS13541 Table 70/71) ------------
 * 13-bit full-byte count ntx split across the two registers; nbtx2:0 in 0x23 is the number
 * of bits in the last (partial) byte, 0 for whole bytes only. */
#define ST25R_NTX1(nbytes) ((uint8_t)(((nbytes) >> 5) & 0xFF))
#define ST25R_NTX2(nbytes, nbits) ((uint8_t)((((nbytes) & 0x1F) << 3) | ((nbits) & 0x07)))

/* ---- Auxiliary display register (0x31, DS13541 Table 98) ------------------------------- */
#define ST25R_AUX_A_CHA (1u << 7)  /* Rx channel used (0 AM, 1 PM) */
#define ST25R_AUX_EFD_O (1u << 6)  /* external field detected */
#define ST25R_AUX_TX_ON (1u << 5)  /* transmission active */
#define ST25R_AUX_OSC_OK (1u << 4) /* crystal oscillator stable */
#define ST25R_AUX_RX_ON (1u << 3)  /* receive decoder enabled */
#define ST25R_AUX_RX_ACT (1u << 2) /* receiving a message */

/* ---- NFCIP-1 passive target definition register (0x08, DS13541 Table 33) ---------------
 * Card-emulation / listen mode. fdel3:0 is the PCD-to-PICC FDT compensation (2 is the
 * recommended value). The d_* bits DISABLE the chip's automatic responses when set, so 0
 * keeps automatic NFC-A anticollision and the NFC-F SENSF_RES on. */
#define ST25R_PT_FDEL_SHIFT 4
#define ST25R_PT_D_AC_AP2P (1u << 3)   /* 1 disables AP2P frame recognition */
#define ST25R_PT_D_212_424 (1u << 2)   /* 1 disables automatic SENSF_RES (NFC-F) */
#define ST25R_PT_D_106_AC_A (1u << 0)  /* 1 disables automatic NFC-A anticollision */

/* ---- PT_Memory layout (DS13541 Table 8), loaded via ST25R_SPI_PT_LOAD_A from location 0 --
 * NFC-A card-emulation responses: NFCID1 (UID), SENS_RES (ATQA) and the SEL_RES (SAK) for
 * each cascade level. 15 bytes cover the NFC-A set. */
#define ST25R_PT_OFF_NFCID1 0 /* locations 0..9: NFCID1 (4 or 7 UID bytes) */
#define ST25R_PT_OFF_SENS_RES 10 /* locations 10,11: SENS_RES (ATQA) */
#define ST25R_PT_OFF_SELR_L1 12  /* location 12: SEL level 1 response (SAK) */
#define ST25R_PT_OFF_SELR_L2 13  /* location 13: SEL level 2 response */
#define ST25R_PT_OFF_SELR_L3 14  /* location 14: SEL level 3 response (RFU for NFC-A) */
#define ST25R_PT_NFCA_LEN 15     /* bytes written for an NFC-A target */

/* ---- Wake-up timer control register (0x32, DS13541 Table 103/104) ----------------------
 * Low-power tag detection: the chip wakes on the timer, measures the antenna amplitude
 * and/or phase, and raises I_wam / I_wph if the change from the reference exceeds the delta.
 * Period = (wut + 1) x step, step 100 ms (wur=0) or 10 ms (wur=1). */
#define ST25R_WUT_WUR (1u << 7)   /* 0: 100 ms step, 1: 10 ms step */
#define ST25R_WUT_SHIFT 4         /* wut2:0 timeout value */
#define ST25R_WUT_WTO (1u << 3)   /* raise an IRQ at every timeout */
#define ST25R_WUT_WAM (1u << 2)   /* measure amplitude at each timeout */
#define ST25R_WUT_WPH (1u << 1)   /* measure phase at each timeout */

/* ---- Amplitude/phase measurement config registers (0x33 amplitude, 0x37 phase; same
 *      layout, DS13541 Table 105/115) ------------------------------------------------------ */
#define ST25R_MEAS_D_SHIFT 4       /* delta (am_d3:0 / pm_d3:0) that triggers the interrupt */
#define ST25R_MEAS_AAM (1u << 3)   /* include the IRQ-causing measurement in auto-averaging */
#define ST25R_MEAS_AEW_SHIFT 1     /* auto-averaging weight (00:4 01:8 10:16 11:32) */
#define ST25R_MEAS_AE (1u << 0)    /* 1: use auto-averaging as reference; 0: use the ref reg */

/* ---- Interrupt registers: main 0x1A (Table 62), timer/NFC 0x1B (Table 63),
 *      error/wake-up 0x1C (Table 64). The masks at 0x16..0x18 share the same bit layout.
 *      Reading 0x1A..0x1D (auto-increment, 4 bytes) returns and clears all pending IRQs. */
/* Main (0x1A) */
#define ST25R_IRQ_OSC (1u << 7)     /* oscillator frequency stable */
#define ST25R_IRQ_WL (1u << 6)      /* FIFO water level */
#define ST25R_IRQ_RXS (1u << 5)     /* start of receive */
#define ST25R_IRQ_RXE (1u << 4)     /* end of receive */
#define ST25R_IRQ_TXE (1u << 3)     /* end of transmission */
#define ST25R_IRQ_COL (1u << 2)     /* bit collision */
#define ST25R_IRQ_RX_REST (1u << 1) /* automatic reception restart */
/* Timer and NFC (0x1B) */
#define ST25R_IRQ_DCT (1u << 7)  /* direct command terminated */
#define ST25R_IRQ_NRE (1u << 6)  /* no-response timer expired */
#define ST25R_IRQ_GPE (1u << 5)  /* general-purpose timer expired */
#define ST25R_IRQ_EON (1u << 4)  /* external field > target activation level */
#define ST25R_IRQ_EOF (1u << 3)  /* external field dropped below level */
#define ST25R_IRQ_CAC (1u << 2)  /* collision during RF collision avoidance */
#define ST25R_IRQ_CAT (1u << 1)  /* minimum guard time expired (field switched on) */
#define ST25R_IRQ_NFCT (1u << 0) /* target: initiator bit rate recognized */
/* Error and wake-up (0x1C) */
#define ST25R_IRQ_CRC (1u << 7)  /* CRC error */
#define ST25R_IRQ_PAR (1u << 6)  /* parity error */
#define ST25R_IRQ_ERR2 (1u << 5) /* soft framing error */
#define ST25R_IRQ_ERR1 (1u << 4) /* hard framing error */
#define ST25R_IRQ_WT (1u << 3)   /* wake-up timer */
#define ST25R_IRQ_WAM (1u << 2)  /* wake-up: amplitude measurement */
#define ST25R_IRQ_WPH (1u << 1)  /* wake-up: phase measurement */
/* Passive target (0x1D, Table 65) — card-emulation / listen mode */
#define ST25R_IRQ_PPON2 (1u << 7)   /* PPON2 field-on waiting timer */
#define ST25R_IRQ_SL_WL (1u << 6)   /* passive target slot-number water level */
#define ST25R_IRQ_APON (1u << 5)    /* active P2P field-on event */
#define ST25R_IRQ_RXE_PTA (1u << 4) /* end of receive; chip handled the auto response */
#define ST25R_IRQ_WU_F (1u << 3)    /* NFC-F 212/424 passive target Active */
#define ST25R_IRQ_WU_AX (1u << 1)   /* passive target Active* state reached */
#define ST25R_IRQ_WU_A (1u << 0)    /* passive target Active state reached (selected) */

/* The four interrupt registers form one 32-bit set, LSB = main (0x1A). */
#define ST25R_IRQ_MAIN(v) ((uint8_t)((v) & 0xFFu))
#define ST25R_IRQ_TIMER_NFC(v) ((uint8_t)(((v) >> 8) & 0xFFu))
#define ST25R_IRQ_ERROR_WUP(v) ((uint8_t)(((v) >> 16) & 0xFFu))
#define ST25R_IRQ_PASSIVE(v) ((uint8_t)(((v) >> 24) & 0xFFu))

/* ---- FIFO status registers (0x1E/0x1F, DS13541 Table 66/67) ---------------------------- */
#define ST25R_FIFO2_B9_B8_SHIFT 6 /* FIFO byte-count MSBs are bits 7:6 of 0x1F */
#define ST25R_FIFO2_UNDERFLOW (1u << 5)
#define ST25R_FIFO2_OVERFLOW (1u << 4)
#define ST25R_FIFO2_LB_SHIFT 1 /* bits in the last byte if incomplete (fifo_lb2:0) */
#define ST25R_FIFO2_LB_MASK (0x7u << 1)
#define ST25R_FIFO2_NP_LB (1u << 0) /* parity bit missing in the last byte */
#define ST25R_FIFO_DEPTH 512

/* ---- Collision display register (0x20, DS13541 Table 68) ------------------------------- *
 * Valid after a bit collision in an ISO 14443-A anticollision frame (not if I_err1 is set). */
#define ST25R_COLL_BYTE_SHIFT 4 /* c_byte<3:0>: full bytes before the collision */
#define ST25R_COLL_BYTE_MASK (0xFu << 4)
#define ST25R_COLL_BIT_SHIFT 1 /* c_bit<2:0>: bits before the collision in that byte */
#define ST25R_COLL_BIT_MASK (0x7u << 1)
#define ST25R_COLL_PB (1u << 0) /* c_pb: the first collision was in a parity bit */

/* ---- IC identity register (0x3F, DS13541 Table 121) ------------------------------------ */
#define ST25R_IC_TYPE_SHIFT 3
#define ST25R_IC_TYPE_MASK (0x1Fu << 3)
#define ST25R_IC_TYPE_ST25R3916B 0x06 /* 00110b; the ST25R3916 (non-B) reads 0x05 */
#define ST25R_IC_REV_MASK 0x07u

#endif
