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
- **Power-up to Ready** (`st25r3916_field_on` / `field_off`): oscillator start with the
  `osc_ok` wait, `Adjust regulators`, NFC-A reader mode at 106 kbit/s, and field-on with
  initial RF collision avoidance and the ISO 14443 guard time (DS13541 §4.2.13).
- **Interrupt-driven transceive** (`st25r3916_transceive`): TX from the FIFO with the
  number-of-transmitted-bytes registers, RX back out of it, CRC / parity / bit-oriented
  anticollision framing flags, and error/collision/CRC reporting.
- **NFC-A activation** (`st25r3916_nfca_request`, `st25r3916_nfca_select`,
  `st25r3916_nfca_halt`): REQA/WUPA and the full ISO 14443-A anticollision + SELECT cascade
  (levels 1–3), returning the UID (4/7/10 bytes), ATQA and SAK. **Several cards in the field**
  are resolved bit by bit (ISO 14443-3 §6.5.3: keep the bits before the collision from the
  Collision display register, choose 1, ask again with the longer prefix); `collided` says
  others answered, and HLTA + select again finds each in turn. The unit test drives it
  against fake cards with 4- and 7-byte UIDs, including collisions on the first and last UID
  bit and two 7-byte cards that only differ at cascade level 2.
- **Other reader protocols and bit rates** (`st25r3916_field_on_tech`, `st25r3916_set_tech`):
  NFC-A and B at 106–848 kbit/s (switch after the caller's PPS / ATTRIB), FeliCa at 212/424.
  `st25r3916_transceive` follows the current protocol.
- **ISO 14443-B** (`nfcb_poll` / `nfcb_attrib` / `nfcb_halt`): REQB/WUPB with AFI, one slot
  and then 16 slots with Slot-MARKERs when answers collide, ATTRIB, HLTB.
- **FeliCa** (`nfcf_poll`): SENSF_REQ with system code, request code and time slots, giving
  IDm, PMm and the system code.
- **ISO 15693** (`nfcv_inventory`, `nfcv_transceive`): the chip has no 15693 framing, so the
  driver codes it in stream mode itself (1-out-of-4 out, Manchester in, SOF/EOF, ISO/IEC
  13239 CRC, checked against the standard `26 01 00 → F6 0A` frame) and finds several cards
  with an inventory mask search. The coding functions `st25r_nfcv_*` are public.
 (`st25r3916_listen_start` / `listen_poll` / `listen_respond` /
  `listen_stop`): act as a tag. The chip answers REQA, anticollision and SELECT automatically
  from PT_Memory (loaded with the NFCID1/ATQA/SAK, 4- or 7-byte UID); the host is notified
  when a reader selects the card, then handles the higher-layer frames (RATS/APDU) over the
  FIFO. Exercised in the unit test against a fake reader.
- **Wake-up mode** (`st25r3916_wakeup_start` / `wakeup_cause` / `wakeup_stop`): low-power tag
  detection. The chip sleeps with the oscillator off and wakes on its own timer to measure
  the antenna amplitude and/or phase, raising an interrupt when a tag detunes the antenna past
  a threshold. Supports a fixed calibrated reference or a drift-tracking auto-average. The
  delta thresholds are antenna-specific and must be tuned on hardware.
- **Analog-config table** (`st25r3916_set_analog_config` / `apply_analog_config`): the
  receiver/modulation/regulator settings depend on your antenna, so you supply them as a table
  of `{event, register, mask, value, space}` tweaks and the RF flows apply the right ones at
  each event (power-up, NFC-A poll, NFC-A listen, wake-up). See
  `examples/analog_config_example.c`.

**Not implemented yet** (contributions welcome — these need the datasheet's analog/RF
sections and, to be trusted, a real chip):

- Ready-made **analog values for a specific antenna** — the table mechanism is here, but the
  numbers are yours to supply and tune (the driver runs on the chip's reset defaults without
  a table).
- Card emulation in protocols other than NFC-A, and Topaz / NFCIP-1 active mode.

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
| `st25r3916_field_on` / `field_off` | Power up to Ready and switch the RF field on/off |
| `st25r3916_transceive` | Send/receive a frame (CRC, parity, anticollision flags) |
| `st25r3916_field_on_tech` / `set_tech` | Reader protocol (A/B/F/V) and bit rate |
| `st25r3916_nfcb_poll` / `nfcb_attrib` / `nfcb_halt` | ISO 14443-B reader |
| `st25r3916_nfcf_poll` | FeliCa polling |
| `st25r3916_nfcv_inventory` / `nfcv_transceive` | ISO 15693 reader (driver-coded stream mode) |
| `st25r3916_nfca_request` / `nfca_select` / `nfca_halt` | REQA/WUPA, the ISO 14443-A UID cascade with collision resolution, HLTA |
| `st25r3916_listen_start` / `listen_poll` / `listen_respond` / `listen_stop` | NFC-A card emulation |
| `st25r3916_wakeup_start` / `wakeup_cause` / `wakeup_stop` | Low-power tag detection (wake-up mode) |
| `st25r3916_set_analog_config` / `apply_analog_config` | Install/apply the antenna's analog settings |

The RF calls expect the chip's IRQ pin wired to an optional `st25r_bus_t.wait_irq` callback
(they poll the interrupt registers if it is `NULL`). They return `ST25R_OK` or a negative
`ST25R_ERR_*`.

Also to confirm on a chip: **FeliCa's LEN byte** is assumed to be added and removed by the
chip (DS13541 §4.4.4 says Transmit without CRC "skips the Length and CRC bytes"; set
`nfcf_host_len` if not), and **the stream-mode format** for ISO 15693 is assumed to be LSB
first, one bit per fc/128 slot out and per 8 subcarrier pulses in (register 0x09 = 0x38),
1 = pulse / subcarrier seen (DS13541 does not describe it).

Two details of the split-byte anticollision frame are not spelled out in DS13541 and are
assumptions to confirm on a real chip: the first received byte is aligned to the split
position, and the Collision display register (0x20) counts from the start of the frame
(SEL). If either is wrong, `nfca_select` returns `ST25R_ERR_FRAMING` rather than a wrong UID.

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
