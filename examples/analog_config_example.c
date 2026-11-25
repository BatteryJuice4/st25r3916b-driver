/*
 * analog_config_example.c — how a board supplies its antenna-specific analog settings.
 *
 * The receiver, modulation and regulator registers that make the RF work well are specific to
 * your antenna and matching network, so the driver does not ship values for them. Instead you
 * build a table of register tweaks and hand it to the driver; it applies the ones for each RF
 * event (power-up, NFC-A poll, NFC-A listen, wake-up) at the right moment.
 *
 * The register addresses and bit meanings are in st25r3916_regs.h (each cited to its DS13541
 * table). The VALUES below are placeholders — start from the datasheet's descriptions and tune
 * them on hardware with your antenna (ST's RFAL reference-antenna numbers are a guide, but they
 * are ST's and are not reproduced here).
 */
#include "st25r3916.h"
#include "st25r3916_regs.h"

/* Each entry: { event, register, mask, value, space_B? }.
 *   applied as  reg = (reg & ~mask) | (value & mask). */
static const st25r_analog_entry_t my_board_analog[] = {
    /* --- chip-level, once the oscillator is ready --- */
    {ST25R_ANALOG_POWER_UP, ST25R_REG_ANT_TUNE1, 0xFF, 0x80, 0}, /* antenna tuning (tune me) */
    {ST25R_ANALOG_POWER_UP, ST25R_REG_ANT_TUNE2, 0xFF, 0x80, 0},

    /* --- NFC-A reader: receiver and modulation --- */
    {ST25R_ANALOG_POLL_A, ST25R_REG_RX_CONF1, 0xFF, 0x08, 0},     /* Rx filter (tune me) */
    {ST25R_ANALOG_POLL_A, ST25R_REG_RX_CONF2, 0xFF, 0x2D, 0},
    {ST25R_ANALOG_POLL_A, ST25R_REG_TX_DRIVER, 0x0F, 0x01, 0},    /* modulation depth (tune me) */
    {ST25R_ANALOG_POLL_A, ST25R_REGB_CORR_CONF1, 0xFF, 0x51, 1},  /* correlator (space B) */

    /* --- NFC-A card emulation: receiver for the reader's field --- */
    {ST25R_ANALOG_LISTEN_A, ST25R_REG_RX_CONF1, 0xFF, 0x08, 0},

    /* --- wake-up measurement front-end --- */
    {ST25R_ANALOG_WAKEUP, ST25R_REG_RX_CONF1, 0xFF, 0x04, 0},
};

static const st25r_analog_config_t my_board_config = {
    my_board_analog, sizeof(my_board_analog) / sizeof(my_board_analog[0])};

void board_nfc_install_analog(st25r_t *dev)
{
    /* After st25r3916_probe(), before the first field_on/listen_start/wakeup_start: */
    st25r3916_set_analog_config(dev, &my_board_config);
}
