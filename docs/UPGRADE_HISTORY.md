# SPI Air-Quality System: Design History

How the design grew from a single-sensor prototype to the current version.
Each section records what changed and why. Numbers inside a section describe
the design as it was at that step; the current values are in the last section
and in the README.

| Step | Main change |
|------|-------------|
| Base | One sensor, 32-bit frame, 2-bit XOR checksum |
| Upgrade 1 | CRC-8 error detection |
| Upgrade 2 | Alarm log in on-chip memory, two-way UART |
| Upgrade 3 | Alarm hysteresis, watchdog, sleep mode |
| v2 | SPI slave register map for an external MCU |
| v3 | Triple redundancy, moving average, CO channel, trend flag, SPI flash log |
| v4 | AQI computed only in the processing slave |
| v5 | Timing closure: registered voter, corrected I/O constraints |

Design conventions used throughout:

- UART runs at 921,600 baud (CLKS_PER_BIT = 109, under 0.5 % error).
- Alarm log entries are 64 bits wide.
- The watchdog is a 10-bit counter, so the timeout needs no wide comparator.

---

## Starting Point — Base System

| Parameter | Value |
|-----------|-------|
| Sensors | 1× spi_sensor_slave (PM2.5, PM10, VOC only) |
| SPI frame | 32-bit (3 × 10-bit data + 2-bit XOR checksum) |
| UART TX | 2-byte output only |
| AQI output | 9-bit |
| Error detection | 2-bit XOR (catches only ~25% of single-bit errors) |
| Redundancy | None — single sensor, single point of failure |

---

## Upgrade 1 — CRC-8 Error Detection

### Why
The original 2-bit XOR checksum catches only ~25% of random single-bit errors.  
CRC-8 (polynomial 0x07, CCITT) catches:
- 100% of all single-bit errors
- 100% of all double-bit errors
- 100% of all burst errors ≤ 8 bits
- ~99.6% of all other error patterns

### Frame Changes

| Field | Old width | New width |
|-------|-----------|-----------|
| PM2.5 | 10 bits | 10 bits (unchanged) |
| PM10 | 10 bits | 10 bits (unchanged) |
| VOC | 10 bits | 10 bits (unchanged) |
| Checksum | 2-bit XOR | 8-bit CRC-8 |
| **Total frame** | **32 bits** | **40 bits** |
| DATA_WIDTH parameter | 32 | 40 |

### SPI Frame Layout (40-bit, MSB-first)
```
[39:30]  PM2.5  (10 bits, range 0–499)
[29:20]  PM10   (10 bits, range 0–599)
[19:10]  VOC    (10 bits, range 0–999)
[ 9: 8]  unused (2-bit pad, always 0)
[ 7: 0]  CRC-8  (polynomial 0x07, init 0xFF)
```

### CRC-8 Algorithm
- Polynomial: 0x07 (x⁸ + x² + x + 1) — standard CCITT CRC-8
- Init value: 0xFF, MSB-first, purely combinational LFSR
- Input: 30 data bits {PM2.5[9:0], PM10[9:0], VOC[9:0]}

### Files Affected
| File | Change |
|------|--------|
| `spi_sensor_slave.v` | Replace 2-bit XOR with CRC-8 LFSR |
| `spi_processing_slave.v` | Recompute CRC-8, compare to received [7:0] |
| `spi_master.v` | DATA_WIDTH 32 → 40 |
| `spi_top.v` | DATA_WIDTH 32 → 40 |
| `spi_tb.v` | Update frame checks for 40-bit |

---

## Upgrade 2 — Alarm History BRAM + Bidirectional UART

### BRAM Specification

| Parameter | Value | Reason |
|-----------|-------|--------|
| Entry width | 64 bits | 2×32-bit — standard BRAM port width |
| Depth | 64 entries | 512 bytes total — fits in any FPGA BRAM |
| Address width | 6 bits | log₂(64) = 6 |
| Write condition | alarm_rising only | Does not fill with routine readings |

### BRAM Entry Layout (64 bits)
```
[63:32]  timestamp   — 32-bit free-running counter (wraps every 42.9 s at 100 MHz)
[31:16]  AQI         — [15:9]=zero pad, [8:0]=AQI value (0–500)
[15:8]   flags       — [7:5]=category[2:0] | [4:2]=input_err[2:0] | [1]=crc_err | [0]=alarm_state
[ 7: 0]  reserved    — 0x00
```

### UART TX Packet Formats

**Live reading (0xAA header, 4 bytes):**
```
Byte 0: 0xAA           sync/header
Byte 1: AQI[15:8]      upper byte
Byte 2: AQI[7:0]       lower byte
Byte 3: {input_err[2:0], crc_err, cat[2:0], alarm_state}
```

**Log dump entry (0xBB header, 8 bytes per entry):**
```
Byte 0: 0xBB           dump entry header
Byte 1: entry_index    0–63
Byte 2-5: timestamp[31:0]
Byte 6: AQI[7:0]
Byte 7: flags[7:0]
```
End-of-dump: single byte `0xDD`

### UART RX Command Set

| Command | Action |
|---------|--------|
| 0x01 | Get latest AQI reading — triggers 4-byte live packet |
| 0x02 | Dump full alarm log — sends all BRAM entries |
| 0x03 | Clear alarm log — resets BRAM pointers, sends 0xCC ACK |
| 0x04 | Wake from sleep — wakeup_o pulses high |

### Files Added / Changed
| File | Status |
|------|--------|
| `alarm_bram.v` | **NEW** — 64×64-bit synchronous circular BRAM |
| `uart_rx.v` | **NEW** — 8N1 receiver + command decoder |
| `spi_top.v` | Add BRAM write logic, command dispatcher FSM |
| `spi_tb.v` | Add RX stimulus tasks, dump verification |

---

## Upgrade 2b — UART Conflict Resolution (Live vs Log Dump)

### The Problem
A full BRAM dump (64 entries × 8 bytes) at 921,600 baud takes ~5.58 ms.  
During that time ~206 new readings are produced. Without a solution, live data is completely blocked.

### Solution — Interleaved Priority Transmission
- Live readings have **HIGH priority**; log dump has **LOW priority** and is pauseable
- After each BRAM entry (8 bytes), the FSM checks `new_reading_pending`
- If a new reading arrived during the dump, it is sent first (4 bytes), then dump resumes
- `rd_ptr` (BRAM read pointer) is persistent — dump resumes exactly where it paused
- Worst-case live delay: 1 BRAM entry = 8 × 10,900 ns = ~87,200 ns

---

## Upgrade 3 — Alarm Hysteresis + Watchdog + Sleep Mode

### Alarm Hysteresis
Prevents alarm chatter on noisy sensor readings:

| Parameter | Value | Meaning |
|-----------|-------|---------|
| N_ON | 4 | 4 consecutive cat ≥ Severe to arm alarm |
| N_OFF | 8 | 8 consecutive cat < Unhealthy to disarm alarm |
| ALARM_ON_THRESH | 3 (Severe) | Minimum category to increment on_ctr |
| ALARM_OFF_THRESH | 2 (Unhealthy) | Below this threshold, off_ctr increments |

`alarm_rising`: 1-cycle pulse at alarm assertion edge — gates BRAM and Flash writes.

### Watchdog Timer
Prevents FSM deadlock if a sensor stops responding:

| Parameter | Value |
|-----------|-------|
| Counter width | 10 bits |
| Timeout | bit[9] = 1 at 512 clocks = 5.12 µs |
| On timeout | All CS lines released, FSM → IDLE, sensor_timeout_latched set |

### Sleep Mode

| Signal | Behaviour |
|--------|-----------|
| sleep_i = 1 | FSM stays in IDLE; SPI/UART TX halted; UART RX still active |
| sleep_i = 0 or cmd 0x04 | wakeup_o pulses HIGH for 1 cycle; normal operation resumes |

---

## Upgrade v2 Track — SPI Slave Register Map

An external MCU can read 9 live system registers over a dedicated SPI slave interface (separate pins from the sensor bus).

### Register Map

| Address | Register | Width | Description |
|---------|----------|-------|-------------|
| 0x00 | PM2.5 | 16-bit | MAF-averaged PM2.5 (0–499 µg/m³) |
| 0x01 | PM10 | 16-bit | MAF-averaged PM10 (0–599 µg/m³) |
| 0x02 | VOC | 16-bit | MAF-averaged VOC (0–999 ppb) |
| 0x03 | CO | 16-bit | MAF-averaged CO (0–999 ppm) |
| 0x04 | AQI | 16-bit | Current AQI (0–500) |
| 0x05 | Category | 8-bit | 0=Good … 5=Hazardous |
| 0x06 | Status | 8-bit | {alarm, crc_err, timeout, sleep, 4'b0} |
| 0x07 | Faults | 8-bit | {5'b0, fault_C, fault_B, fault_A} |
| 0x08 | Trend | 8-bit | {7'b0, trend_rising} |

### Protocol
- 16-bit MSB-first transaction: bit[7]=R/W, bits[6:0]=address
- All register inputs double-flopped for CDC protection
- Registers 0x00–0x04 are 16-bit; 0x05–0x08 are 8-bit

### Files Added
| File | Status |
|------|--------|
| `spi_slave.v` | **NEW** — register-based SPI slave |
| `spi_top.v` | Instantiate spi_slave, wire live registers |

---

## Upgrade v3 — Redundancy, Filtering and Persistent Logging

### v3 Track 1A — Triple Modular Redundancy (TMR)

Replace single sensor with three identical `spi_sensor_slave` instances. A combinational median-of-three voter selects the representative value per channel.

```
Sensor A ──┐
Sensor B ──┼──► median3(a,b,c) ──► Validated Output per channel
Sensor C ──┘
```

**Fault detection:** If any sensor's PM2.5 deviates from the median by more than `FAULT_THRESH = 50`, its `sensor_fault_o` bit is set.

New chip selects: cs3 (Sensor B), cs4 (Sensor C).  
New ports: miso_b_i, miso_c_i, raw_pm25_b_i ... raw_co_c_i

### v3 Track 1B — Moving Average Filter (MAF, N=8)

8-entry circular buffer per channel (PM2.5, PM10, VOC, CO, AQI).  
Running 14-bit accumulator; average = acc >> 3.  
Pre-filled with the first valid reading to prevent startup ramp.  
Separate AQI MAF used for trend detection.

### v3 Track 1C — CO (Carbon Monoxide) Channel

Fourth pollutant added; SPI frame expanded from 40 → 50 bits.

**New 50-bit frame layout:**
```
[49:40]  PM2.5  (10 bits, 0–499)
[39:30]  PM10   (10 bits, 0–599)
[29:20]  VOC    (10 bits, 0–999)
[19:10]  CO     (10 bits, 0–999 ppm)
[ 9: 8]  unused (2-bit pad)
[ 7: 0]  CRC-8  (over all 40 data bits)
```
CRC-8 now covers 40 data bits: {PM2.5, PM10, VOC, CO}.

### v3 Track 2 — AQI Trend Detection

```
trend_rising_o = (aqi_latched − avg_aqi) > TREND_THRESHOLD (default 20)
```
Early warning signal when AQI is rising faster than the 8-sample moving average.  
Exposed via SPI slave register 0x08.

### v3 Track 4 — Fault Injection Verification Testbench

| Test | What is verified |
|------|-----------------|
| T18 | Single bit-flip on MISO → CRC-8 detects it |
| T19 | 8-bit burst error → CRC-8 detects it |
| T20 | Sensor no-response → watchdog fires, FSM recovers to IDLE |
| T21 | Sensor dropout then recovery — normal operation resumes |
| T22 | Sensor B PM2.5 spike → sensor_fault[1] set, median unaffected |

### v3 Track 5 — SPI Flash Persistent Logging

`spi_flash_ctrl` implements W25Q128-compatible Write Enable (0x06), Page Program (0x02), and Read Data (0x03) sequences.

Each alarm record: 8 bytes at byte address N×8 (up to 512 records in 4 KB).

**Flash record format:**
```
Byte 0–3: timestamp[31:0]
Byte 4–5: AQI[15:0]
Byte 6:   flags {category[2:0], input_err[2:0], crc_err, alarm_state}
Byte 7:   0x00 reserved
```

UART command 0x06 triggers flash dump; 9-byte packets with 0xBF header are transmitted via UART.

**Flash dump packet (0xBF header, 9 bytes):**
```
Byte 0: 0xBF    header
Byte 1: index   record number
Byte 2–5: timestamp[31:0]
Byte 6: AQI[15:8]
Byte 7: AQI[7:0]
Byte 8: flags
```

New UART RX command added: 0x06 → cmd_flash_dump_o

### Files Added / Changed in v3

| File | Status | Change |
|------|--------|--------|
| `spi_sensor_slave.v` | Modify | CO channel added, 50-bit frame, CRC over 40 bits |
| `spi_processing_slave.v` | Modify | CO AQI path, 50-bit receive, CRC over 40 bits |
| `spi_master.v` | Modify | DATA_WIDTH 40 → 50 |
| `spi_top.v` | Major | TMR, MAF, CO, Trend, Flash ctrl, all new states |
| `spi_slave.v` | Modify | CO register (0x03), Trend register (0x08) |
| `spi_flash_ctrl.v` | **NEW** | W25Q128-compatible SPI NOR flash controller |
| `spi_flash_model.v` | **NEW** | Behavioural simulation model (not synthesizable) |
| `uart_rx.v` | Modify | Add 0x06 flash dump command |
| `spi_tb.v` | Major | All new test sections H, I, J, K |
| `spi_top.sdc` | Modify | Timing constraints for new ports |
| `run_synth.tcl` | Modify | Add new source files |

---

## Final State Summary — Upgrade v3

| Feature | Value |
|---------|-------|
| SPI frame width | 50 bits (DATA_WIDTH=50) |
| Pollutant channels | PM2.5, PM10, VOC, CO |
| Error detection | CRC-8 poly 0x07, init 0xFF, 40-bit coverage |
| Sensor redundancy | TMR (3× sensors, median voter, fault flags) |
| Averaging | MAF N=8 per channel + AQI MAF for trend |
| Alarm | Hysteresis N_ON=4 / N_OFF=8 |
| Watchdog | 10-bit, 512-clock timeout (5.12 µs) |
| Sleep/wake | sleep_i pin + 0x04 UART command |
| MCU readback | SPI slave, 9 registers |
| On-chip log | 64×64-bit BRAM circular buffer |
| Persistent log | SPI NOR Flash, 8-byte records, up to 512 entries |
| UART baud | 921,600 baud (CLKS_PER_BIT=109) |
| Test coverage | 31 test cases across 11 sections (A–K) |
| Synthesis | Cadence Genus / 90nm, SDC constraints provided |


---

## Upgrade v4 — Processing-Centric Architecture Refactor

### Why
In v3, AQI was computed independently in two places: `spi_top` calculated a backup
`med_aqi_out` directly from the median-voted channels, while `spi_processing_slave`
separately verified CRC-8 on the raw Sensor A frame. This meant two different
modules could disagree on the AQI value, and the processing slave never saw the
benefit of TMR voting or MAF smoothing — it only checked Sensor A's frame.

v4 removes the dual-path ambiguity: `spi_processing_slave` becomes the **sole AQI
computation authority**, and it now receives the fully validated, TMR-voted,
MAF-smoothed data instead of a raw single-sensor frame.

### Architectural Changes

| Aspect | v3 | v4 |
|--------|----|----|
| AQI computation | Computed in both `spi_top` (backup) and `spi_processing_slave` | Computed **only** in `spi_processing_slave` |
| Frame sent to processing slave | Raw Sensor A frame | New **Internal Transfer Frame**: MAF-averaged concentrations + fresh CRC-8 |
| SPI SCLK | 12.5 MHz (clk_i ÷ 8) | 25 MHz (clk_i ÷ 4) |
| FSM state count | 58 states | 23 states |
| CRC-fail handling | Sensor CRC failure not substituted before voting | Failed sensor's channel values replaced with **11-bit widened midpoint** of the other two before entering the median voter, preventing overflow at max readings |
| CRC checkpoints | One (raw Sensor A frame, verified in processing slave) | Two independent checkpoints: (1) each sensor frame verified in `CRC_CHK_A/B/C` states, (2) the new averaged frame verified again inside `spi_processing_slave` |
| Alarm/trend evaluation | Used registered/latched AQI | Uses `proc_aqi` / `proc_cat` **live wires directly**, avoiding a one-cycle lag |

### New/Renamed FSM States
`CRC_CHK_A`, `CRC_CHK_B`, `CRC_CHK_C` — register each sensor's CRC pass/fail result
immediately after that sensor's frame is captured.

`MAF_UPDATE` — separated out from the old inline median-vote state; updates all
four 8-sample circular buffers.

`BUILD_PROC_FRAME` — packs the four MAF-averaged channel values into a new 50-bit
frame and computes a fresh CRC-8 over it, before forwarding to the processing slave.

### Internal Transfer Frame (New in v4)
```
[49:40] avg_pm25   [39:30] avg_pm10   [29:20] avg_voc
[19:10] avg_co     [9:8] reserved     [7:0] CRC-8 (fresh, over averaged data)
```
This is distinct from the original per-sensor frame — it carries post-TMR,
post-MAF data, not raw ADC-derived values.

### Files Changed in v4
| File | Change |
|------|--------|
| `spi_top.v` | Removed backup AQI computation; added CRC_CHK_A/B/C, MAF_UPDATE, BUILD_PROC_FRAME states; added 11-bit widened CRC-fail substitution; SCLK divider changed ÷8 → ÷4 |
| `spi_processing_slave.v` | Now the sole AQI authority; validates the *averaged* frame's CRC instead of the raw sensor frame; 2-stage pipeline (segment select → multiply-accumulate) added to meet timing |
| `spi_top.sdc` | Updated generated-clock divide ratio for the new 25 MHz SCLK |
| `spi_top_props.sv` | Added Property 14 — checks `aqi_latched == proc_aqi` and `cat_latched == proc_cat` at LATCH_DATA, enforcing single-source-of-truth AQI |

### Final State Summary — Upgrade v4

| Feature | Value |
|---------|-------|
| AQI computation authority | `spi_processing_slave` only (no dual path) |
| SPI sensor bus SCLK | 25 MHz (clk_i ÷ 4) |
| Main FSM states | 23 |
| CRC checkpoints | 2 independent (per-sensor + averaged-frame) |
| CRC-fail substitution | 11-bit widened midpoint of surviving pair |
| Formal properties | 14 (added AQI/category consistency check at LATCH_DATA) |
| Test coverage | 60 tests across 12 sections (A–M) |

---

## Upgrade v5 — Timing Closure

### Why
Static timing analysis after place and route showed two problems:

1. **Hold violations on input paths.** The worst three were the MCU SPI pins
   (`sclk_s_i`, `cs_s_i`, `mosi_s_i`), each wired straight to the first flop of
   a two-flop synchroniser. These pins are asynchronous to `clk_i`, so there is
   nothing to time, but the constraints applied a `clk_i`-referenced input
   delay to every input. The same constraints also referenced all I/O delays to
   the real clock at its port, so once the clock tree existed every short input
   path was checked against a clock edge that arrives a full insertion delay
   later than the launch edge.
2. **Very little setup margin.** The longest path ran from the sensor frame
   registers through the CRC-fail substitution adders, the median compare tree
   and the moving-average adders in a single cycle, even though the FSM already
   spends a state (`MEDIAN_VOTE`) on the vote.

### RTL changes

| File | Change |
|------|--------|
| `spi_top.v` | Median voter outputs (and the substituted PM2.5 values used for fault detection) are registered in `MEDIAN_VOTE`. `MAF_UPDATE` and the sensor-fault comparators use the registered values. No extra latency: the state already existed. |
| `spi_top.v` | `LATCH_DATA` can last several cycles when the UART FIFO is busy, while the processing slave holds its result only briefly. The AQI latch, alarm counters, AQI moving average and trend flag now sample in the first `LATCH_DATA` cycle only. A live packet that has to wait is built from the latched values. |
| `spi_top.v` | `trend_rising_o` is evaluated once per measurement and held, so the MCU reads a stable level from register 0x08. `sensor_fault_o` no longer changes while a new set of frames is still being read. |
| `spi_processing_slave.v` | Removed an unused 50-bit register. |
| `spi_top_props.sv` | Property 14 now checks the latched AQI one clock after entry to `LATCH_DATA`, matching when the latch happens. |

### Constraint changes

| Item | Before | After |
|------|--------|-------|
| I/O delay reference | `clk_i` at the port | Virtual clock `clk_io` with latency equal to the clock insertion delay |
| Clock ports | Included in `all_inputs` input delay | Excluded |
| `sclk_s_i` | Declared as a 10 MHz clock, although no flop is clocked by it | Treated as an asynchronous data input |
| MCU SPI pins | Timed against `clk_i` | False path, like the other synchronised inputs |
| Processing-slave multicycle exception | 2-cycle setup | Removed; the path closes in one cycle |

### Result (OpenROAD, sky130hd, after clock tree synthesis, before any hold repair)

| Metric | v4 | v5 |
|--------|----|----|
| Hold violations | 27 endpoints, worst slack -0.83 ns | none, worst slack +0.01 ns |
| Worst register-to-register setup slack | +0.51 ns | +1.19 ns |
| Testbench | 60 tests, 146 checks, all pass | 60 tests, 146 checks, all pass |
