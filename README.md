# st25r3916b-driver

A small, MCU-independent C driver for the **STMicroelectronics ST25R3916B** 13.56 MHz NFC
reader / front end (and its siblings ST25R3917B / ST25R3919B, which share the host
interface). Written from the datasheet, with no dependency on ST's RFAL middleware.

The driver is deliberately portable: it never talks to your SPI peripheral directly. You
give it **one callback** that does a single chip-select-framed SPI transaction, and the
driver builds every register, FIFO, interrupt and command access on top of that. The same
code compiles for a PC (where the included unit test drives a fake chip) and for any
microcontroller.

## Status

> ⚠️ **This is the register / command / SPI foundation, and it is untested on real
> hardware.** The author does not yet have a board with the chip.

**Implemented and unit-tested (against a fake chip):**

- SPI framing per DS13541 §4.3.3 (mode 1, MSB first): register read/write with address
  auto-increment, register **space-B** access (the `0xFB` prefix), FIFO load/read.
- All direct commands (`0xC0`–`0xFC`) and the full register map, in `st25r3916_regs.h`,
  each value referenced to its datasheet table.
- Interrupt handling: read-and-clear the four interrupt registers in one burst, and set the
  four mask registers, via a packed 32-bit value.
- Device identification: `st25r3916_probe()` reads the IC identity register and confirms the
  part is an ST25R3916B.

**Not implemented yet** (contributions welcome — these need the datasheet's analog/RF
sections and, to be trusted, a real chip):

- Power-up sequence (oscillator start, `Adjust regulators`) and the analog-config register
  values (those depend on your antenna and matching network).
- NFC-A polling / ISO 14443-A anticollision.
- Interrupt-driven transceive (TX from the FIFO, RX with CRC/parity handling).
- Card emulation (target / listen mode).
- Low-power card detection (wake-up mode).

## Using it

Add `src/st25r3916.c` to your build and put `include/` on your include path. Implement an
`st25r_bus_t` for your SPI peripheral:

```c
#include "st25r3916.h"

// One chip-select-framed, full-duplex SPI transaction: drive CS/BSS low, clock n bytes out
// of tx while reading n into rx (either may be NULL), then drive CS high. SPI mode 1, MSB first.
static void my_spi_txn(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n) {
    my_cs_low();
    my_spi_transfer(tx, rx, n);   // your HAL's full-duplex transfer
    my_cs_high();
}

static void my_delay_us(void *ctx, unsigned us) { my_busy_wait_us(us); }

static const st25r_bus_t bus = { .ctx = NULL, .txn = my_spi_txn, .delay_us = my_delay_us };

void nfc_init(void) {
    st25r_t dev = { .bus = &bus };
    if (!st25r3916_probe(&dev)) {
        // no ST25R3916B on the bus
        return;
    }
    // dev.ic_type / dev.ic_rev now hold the identity; build your RF flows from here.
}
```

See `examples/bus_example.c` for the same pattern spelled out, and `test/test_st25r3916.c`
for a complete fake bus that emulates the register file.

### API

`st25r3916.h` is the whole public interface. In short:

| Call | Does |
|---|---|
| `st25r3916_probe` | Read IC identity; true if it's an ST25R3916B |
| `st25r3916_read_reg` / `write_reg` / `modify_reg` | Space-A register access |
| `st25r3916_read_regs` | Burst read with address auto-increment |
| `st25r3916_read_regb` / `write_regb` | Space-B register access (`0xFB` prefix) |
| `st25r3916_write_fifo` / `read_fifo` / `fifo_count` | 512-byte FIFO |
| `st25r3916_get_interrupts` / `set_interrupt_mask` | The four interrupt / mask registers |
| `st25r3916_cmd` | Any direct command (`ST25R_CMD_*`) |

No dynamic allocation. The calls are safe to use from an ISR if your bus callback is.

## Build and test

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

The library itself needs only a C11 compiler. Tests build on any host.

## Datasheet

Register addresses, command codes and bit fields are transcribed from ST's datasheet
**DS13541** (ST25R3916B / ST25R3917B / ST25R3919B) — get it from
[st.com](https://www.st.com/en/nfc/st25r3916b.html). The code cites the relevant tables so
every constant can be checked against the document.

## Licensing and trademarks

This driver is MIT-licensed (see `LICENSE`). It is an independent implementation written
from the public datasheet; it contains no ST source code and does not use or derive from
ST's RFAL library. Register maps and command codes are interface facts, not ST's software.

"ST25R3916B", "ST25R3917B" and "ST25R3919B" are STMicroelectronics products; "NFC Forum",
"MIFARE", "FeliCa" and related marks belong to their respective owners. This project is not
affiliated with or endorsed by STMicroelectronics. If you build a product with this chip,
RF emissions certification (FCC Part 15, ETSI EN 300 330, etc.) and any applicable patents
are your responsibility.

## Contributing

The RF flows listed under "Not implemented yet" are the obvious next steps. If you have a
board and a logic analyzer, reports of what works (and corrections to any datasheet value)
are especially valuable — open an issue or a pull request.
