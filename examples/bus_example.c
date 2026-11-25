/*
 * bus_example.c — how to connect the driver to a real SPI peripheral.
 *
 * This is illustrative pseudocode: replace the my_* calls with your MCU HAL. It does not
 * compile on its own (there is no my_spi.h); it shows the one thing you must provide — an
 * st25r_bus_t whose txn() does a single chip-select-framed, full-duplex SPI transaction in
 * SPI mode 1 (clock polarity 0, clock phase 1), MSB first.
 */
#include "st25r3916.h"

/* ---- Replace these with your platform's SPI + GPIO + timing ---------------------------- */
extern void my_cs_low(void);
extern void my_cs_high(void);
extern void my_spi_transfer(const uint8_t *tx, uint8_t *rx, size_t n); /* full-duplex */
extern void my_busy_wait_us(unsigned us);

/* One transaction: CS low, clock n bytes each way, CS high. tx or rx may be NULL. */
static void example_txn(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n)
{
    (void)ctx;
    my_cs_low();
    my_spi_transfer(tx, rx, n);
    my_cs_high();
}

static void example_delay_us(void *ctx, unsigned us)
{
    (void)ctx;
    my_busy_wait_us(us);
}

static const st25r_bus_t example_bus = {
    .ctx = 0,
    .txn = example_txn,
    .delay_us = example_delay_us,
};

int example_main(void)
{
    st25r_t dev = {.bus = &example_bus};

    if (!st25r3916_probe(&dev)) {
        return -1; /* no ST25R3916B answered on the bus */
    }

    /* Put the chip in its reset state, then enable the oscillator and regulator (Ready). */
    st25r3916_cmd(&dev, ST25R_CMD_SET_DEFAULT);
    st25r3916_modify_reg(&dev, ST25R_REG_OP_CONTROL, ST25R_OP_EN, ST25R_OP_EN);

    /* From here you would wait for the oscillator-stable interrupt, adjust the regulators,
     * load your analog configuration, and build the RF flows your application needs. Those
     * steps are not provided by this driver yet (see README.md). */
    return (int)dev.ic_rev;
}
