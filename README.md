# zmk-iqs7211e-driver

[![Test](https://github.com/amgskobo/zmk-iqs7211e-driver/actions/workflows/test.yml/badge.svg)](https://github.com/amgskobo/zmk-iqs7211e-driver/actions/workflows/test.yml)

[日本語](README_JA.md)

## 1. Overview

This repository provides an Azoteq IQS7211E touch/proximity sensor driver for ZMK (Zephyr Mechanical Keyboard firmware). It has been verified with **Zephyr 4.1**.
The driver is inspired by the [ZMK PMW3610 driver](https://github.com/inorichi/zmk-pmw3610-driver). It supports standard ZMK interrupt-driven input and single-finger gesture reporting; board-specific panel capabilities and dimensions belong in the board documentation.

The driver implements touch gestures and reports sensor input without owning
keymap layers or pointer regions:

- Single-tap / Double-tap / Triple-tap
- Precise rotation correction (`rotate-cw`) for flexible physical placement.
- Edge sliders can be added downstream with
  [`zmk-input-temp-layer-touch`](https://github.com/amgskobo/zmk-input-temp-layer-touch),
  including independent left and right pads and delayed-tap suppression.
- "Ultimate Quality" Rigor: Implemented mathematical boundary fixes (Off-by-one) and safe PM (Power Management) execution guards.

## 2. Device Tree Properties

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `reg` | byte | 0x56 | I2C address of the device (required) |
| `irq-gpios` | phandle-array | | Interrupt pin (required)|
| `single-tap` | int | -1 | Button triggered by single-tap (-1=disabled, 0=BTN_0, 1=BTN_1, 2=BTN_2, ...) |
| `double-tap` | int | -1 | Button triggered by double-tap (-1=disabled, 0=BTN_0, 1=BTN_1, 2=BTN_2, ...) |
| `triple-tap` | int | -1 | Button triggered by triple-tap (-1=disabled, 0=BTN_0, 1=BTN_1, 2=BTN_2, ...) |
| `rotate-cw` | uint | 0 | **CW Rotation angle to match physical placement** (0=0°, 1=90°, 2=180°, 3=270°). Coordinates are normalized before they reach downstream input processors. |
| `report-abs` | boolean | false | If true, report absolute coordinates instead of relative ones. |
| `jitter-deadband` | int | 8 | Per-axis coordinate distance held by the rubber-band jitter gate. X and Y are gated independently, not by Euclidean distance. 0 disables the gate and keeps the three-sample median filter. Treat the default as a conservative starting point and override it after measuring the panel. Valid range: 0-1024. |
| `touch-verify-interval-ms` | int | 120 | In both report modes, independently run a full report read at this interval while touch remains active. This provides common stale-contact recovery and is not layer-gated. Set to 0 to use the sensor's 60-second fallback instead. |

### 2.1 Absolute Pointer Report Mode

By default, this driver reports relative coordinates (`INPUT_REL_X`, `INPUT_REL_Y`). By setting `report-abs;` in the Device Tree, it will switch to absolute coordinates (`INPUT_ABS_X`, `INPUT_ABS_Y`).
This is useful when combined with ZMK input processors that expect absolute data, such as a digitizer-to-mouse converter.
The absolute coordinates are reported in the range of 0 to 1024 (as defined by the chip's resolution).

Direct relative reporting does not emit `INPUT_BTN_TOUCH`: ZMK maps that code
to mouse button 0, which would otherwise turn every contact into a left-button
drag. Absolute mode does emit the contact edges because absolute-coordinate
processors use them to delimit a contact. Such a processor must consume or
suppress `INPUT_BTN_TOUCH` before it reaches ZMK's mouse output; for example,
use `suppress-btn-touch` with an absolute-to-relative processor. Tap gestures
remain ordinary `INPUT_BTN_*` click reports in both modes.

Both absolute and relative reporting use the same stateful coordinate filter: a configurable
rubber-band deadband followed by a three-sample median. The driver relies on the sensor's
on-chip MAV and Dynamic IIR rather than applying a second IIR on the host. All filter state is
reset at the start of every contact, and a temporarily invalid coordinate holds the previous
output without advancing the filter. The default deadband is 8; a board
only needs an override when its measurements call for it.

### 2.2 Touch Verification

`touch-verify-interval-ms` independently checks that the physical touch is still present in both report modes. IQS7211E Event Mode can stop producing interrupts while a finger is held still, so this full report read detects a physical release or I2C failure even when no new edge arrives. If the read fails or the sensor reports no fingers, the driver releases the stale contact. The default is 120 ms; set it to 0 only if the sensor's slower 60-second fallback is deliberately preferred.

Every full report also checks the sensor's `Show Reset` flag. If the IQS7211E
watchdog or a sensor-only power interruption resets the part after start-up,
the driver first releases any active click and touch,
then restores the application settings, acknowledges the reset, runs ATI, and
re-enables Event Mode. No coordinates or gestures from the reset packet are
forwarded.

The check is a full report read, not a bare `INFO_FLAGS` poll. A partial read followed by a STOP would close a communication window that had been opened for a gesture or coordinate event and lose it, so the verify goes through the normal report path with the interrupt masked. A verify tick can therefore also emit coordinates and dispatch clicks, exactly as an ordinary report does.

When host verification is active, the driver programs the chip's Idle-Touch timeout to 0 so the chip cannot reseed underneath a touch the host still owns. When the verify interval is 0, the chip's 60-second timeout remains enabled as the stuck-touch fallback in either report mode.

### 2.3 Configuration Design Procedure

Choose configuration in this order instead of selecting isolated values first.

| Step | Decide | Properties | Selection rule and constraint |
|---|---|---|---|
| 1 | Wire connection | `reg`, `irq-gpios` | Required. Use the board I2C address and RDY interrupt GPIO from the schematic. |
| 2 | Tap gestures | `single-tap`, `double-tap`, `triple-tap` | `-1` disables a gesture; `0` through `2` select `BTN_0` through `BTN_2`. Leave unneeded gestures disabled. |
| 3 | Physical orientation | `rotate-cw` | Select `<0>` through `<3>` for the installed orientation. Input-processor edges use the coordinates after this rotation. |
| 4 | Coordinate mode | `report-abs` | Omit for a direct relative pointer. Set `report-abs;` for processors that consume absolute coordinates, such as absolute-to-relative, padstick, or matrix. That downstream path must consume or suppress `INPUT_BTN_TOUCH`. |
| 5 | Noise boundary | `jitter-deadband` | Measure from the default `<8>`. Raise it for stationary jitter, lower it if fine movement is lost. `0` disables only the deadband; the median filter remains. |
| 6 | Edge routing | downstream input processors | Use `zmk-input-temp-layer-touch` for side sliders. Put it before absolute-to-relative conversion on every route. |
| 7 | Contact liveness | `touch-verify-interval-ms` | Normally retain `<120>`. It independently releases stale contacts after a read failure or no-finger report. Set `<0>` only when deliberately relying on the sensor's 60-second fallback. |

After choosing the design, exercise normal pointer movement, every configured
input-processor edge, every tap, a delayed tap after layer release, a stationary
contact, and suspend/resume on hardware.

### 2.4 Filter Tests

The standalone host tests cover deadband behavior, spike rejection, contact reset, invalid
frames, zero-delta relative velocity behavior, absolute/relative coordinate parity, runtime
reset flag detection, and the mode-specific `INPUT_BTN_TOUCH` policy. The parity scenario derives each
expected relative delta independently from the absolute filtered coordinate stream:

```sh
bash tests/run-host-docker.sh
```

CI runs optimized, ASan/UBSan and gcov variants. It requires 100% line and
branch coverage of `iqs7211e_filter.c` and of ten extracted fault/report
functions from `iqs7211e.c`. The driver functions are measured separately
from the host harness; this is not coverage of the entire I2C/IRQ driver.

Implementation and maintenance notes for the coordinate pipeline are included in the
[Japanese README](README_JA.md#5-座標パイプライン).

## 3. Installation (GitHub Actions)

> **Note:** Only GitHub Actions builds are covered here. Local builds differ per user and are not covered.

### 3.1 Add Driver via `west` Manifest

Include this driver in your ZMK repository’s `config/west.yml`:

```yaml
manifest:
  remotes:
    ...
    # START #####
    - name: amgskobo
      url-base: https://github.com/amgskobo
    # END #######
  projects:
    ...
    # START #####
    - name: zmk-iqs7211e-driver
      remote: amgskobo
      revision: main
    # END #######
```

This ensures GitHub Actions pulls the **IQS7211E driver** automatically during the build.

### 3.2 Configure Device Tree Overlay

Add the IQS7211E node in your keyboard DTS overlay file:

```dts
#include <input/processors.dtsi>
#include <dt-bindings/zmk/input_transform.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <dt-bindings/zmk/keys.h>

&pinctrl {
    i2c0_default: i2c0_default {
        group1 {
            psels = <NRF_PSEL(TWIM_SDA, 0, 28)>,
                    <NRF_PSEL(TWIM_SCL, 0, 29)>;
            bias-pull-up;
        };
    };

    i2c0_sleep: i2c0_sleep {
        group1 {
            psels = <NRF_PSEL(TWIM_SDA, 0, 28)>,
                    <NRF_PSEL(TWIM_SCL, 0, 29)>;
            low-power-enable;
        };
    };
};

&i2c0 {
    status = "okay";
    compatible = "nordic,nrf-twi";
    pinctrl-0 = <&i2c0_default>;
    pinctrl-1 = <&i2c0_sleep>;
    pinctrl-names = "default", "sleep";
    clock-frequency = <I2C_BITRATE_FAST>;
    iqs7211e: iqs7211e@56 {
        compatible = "azoteq,iqs7211e";
        reg = <0x56>;
        irq-gpios = <&gpio1 15 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;

        /* Tap gesture settings */
        single-tap = <0>;
        double-tap = <0>;
        triple-tap = <0>;

        rotate-cw = <0>;
        // report-abs; // Use absolute coordinates (0-1024 inclusive)
        // touch-verify-interval-ms = <120>; // optional: layer-independent touch verify
    };
};

/ {
    trackpad_input_listener: trackpad_input_listener {
        compatible = "zmk,input-listener";
        status = "okay";
        device = <&iqs7211e>;
        /* Driver handles the rotation; processors handle the performance/feel */
        input-processors = <&zip_xy_scaler 1 1>;
        scroller {
            layers = <1>;
            input-processors = <&zip_xy_scaler 1 20>,
                               <&zip_xy_to_scroll_mapper>;
        };
    };
};
```

### 3.3 Enable Driver in Kconfig

Add the driver to your `board.conf`:

```kconfig
CONFIG_I2C=y
CONFIG_GPIO=y
CONFIG_INPUT=y
CONFIG_ZMK_POINTING=y
CONFIG_IQS7211E=y
```

The driver uses one private work queue for all IQS7211E instances. Sensor
reports, generated click edges, touch verification, and suspend-time releases
all run there. This is required for reliable input
delivery: Zephyr's asynchronous input backend may make reports from the system
work queue non-blocking, so a full input queue could otherwise drop a release.

After re-enabling the RDY interrupt, the driver also performs a delayed logical
level check. If RDY became active while its edge interrupt was masked, the
report work is queued again without waiting for another edge. Fast recovery is
bounded and then backed off, so a stuck RDY pin cannot spin the work queue.
If the I2C bus is not ready when device PM first tries to wake the sensor, the
same delayed work retries the wake and backs off until the bus recovers.
If device PM resumes the driver before the GPIO controller is ready, the work
also retries arming the RDY interrupt. An inactive RDY level is not considered
recovered until the interrupt has actually been enabled, so the first touch
after deep sleep cannot be left without an event source.

The sensor keeps its power through everything that restarts only the SoC -
the reset button, a firmware update, the watchdog, and a wake from System OFF -
and with it whatever state it was in. That is event mode, where it raises RDY
only for a touch, or the suspend that ZMK's idle sleep put it in (that path
suspends devices in reverse init order, so the bus is still up), where it never
raises RDY at all. Setup starts at the first RDY, so the driver does not wait
for one: if the sensor has raised nothing 200 ms after boot, it forces a
communication window and resets the sensor, which also clears a leftover
suspend. The reset's own RDY then runs setup, ATI included, at boot. Left to
the first touch, the ATI would run under that finger, and the reference taken
there reports a contact that never lifts; a suspended sensor would leave the
pad dead until its power was removed. A freshly powered sensor raises RDY by
itself well within the 200 ms and is left alone.

The sensor is put to sleep on both of ZMK's power-off paths. The sensor's own
device comes after the I2C bus in init order, which is right for idle sleep's
reverse-order suspend but leaves `&soft_off`, which suspends in init order,
with the bus already down when the sensor's turn comes - so the sensor used to
stay awake, at its active current, for as long as the keyboard was off. A small
device ahead of the bus (`CONFIG_IQS7211E_PM_EARLY_INIT_PRIORITY`, 45) runs the
same suspend first on that path; the sensor's own turn then finds it asleep and
skips the write rather than waiting out a bus timeout.

The production defaults are normally sufficient:

```kconfig
CONFIG_IQS7211E_WORKQUEUE_STACK_SIZE=1536
CONFIG_IQS7211E_WORKQUEUE_PRIORITY=-1
```

For a diagnostic build, enable
`CONFIG_IQS7211E_WORKQUEUE_STACK_USAGE=y`. The driver then logs a new stack
high-water mark when peak usage increases. Measure representative movement,
tap sequences, a held contact with touch verification, and suspend/resume before
reducing the stack size. The diagnostic option is disabled by default.

### 3.4 Build Firmware

Push your changes to your GitHub repository.
The GitHub Actions workflow automatically builds the firmware and generates artifacts (`.uf2` or `.bin`) ready for download.

### 3.5 ZMK Studio

The driver supports ZMK Studio, including Studio layer reordering: its layer
properties use stable keymap layer IDs rather than Studio's displayed order.
The trackpad configuration and input-processor routing remain firmware
configuration, while Studio edits ordinary key bindings at runtime. See
[Using the IQS7211E driver with ZMK Studio](docs/zmk-studio.md) for the exact
keymap, GitHub Actions, and validation setup.

## 4. HW and Dimensions

### 4.1 Pin Assignment (all +3V3 logic)

| PIN | value | info |
|-----|-------|------|
|1  |  GND |  - |
|2  |  GND |  - |
|3  |  RDY | irq interrupt pin |
|4  |  +3V3 | VDD |
|5  |  SDA | i2c data|
|6  |  SCL | i2c clock |

### 4.2 BOMs

| Property | Value | Type | Qty | Link |
|----------|------|---------|-------------|-----|
| `C1,C3,C5` | 100pF | 0805_SMD | 3 | |
| `C2,C4` | 2.2uF | 0805_SMD | 2 | |
| `C6` | 4.7uF | 0805_SMD | 1 | |
| `C7` | 100nF | 0805_SMD | 1 | |
| `R1,R2,R3` | 4.7k | 0805_SMD | 3| |
| `J1` | PinHeader_2x03_P2.54mm_Vertical | 2x3pin 2.54mm pitch PH3.5mm height| 1 | [aliexpress](https://ja.aliexpress.com/item/1005003263426999.html) |
| `U1` | IQS7211E001QNR |  IQS7211E001QNR(20-QFN)| 1| [digikey](https://www.digikey.jp/en/products/detail/azoteq-pty-ltd/IQS7211E001QNR/18627341)|

### 4.3 PCB Specifications

The PCB used with this driver is a 2-layer FR4 board with a standard thickness of 1.6 mm. The recommended finish for the PCB is ENIG (Electroless Nickel Immersion Gold).

The ENIG finish provides high durability for the edges of the trackpad and connector areas, allowing for long-term stable use. In addition, the gold layer prevents oxidation, ensuring stable touch sensitivity and response.

Please note that if the PCB thickness is different from 1.6 mm, it may affect the installation and feel of the trackpad. Also, the ENIG finish may incur higher costs compared to standard finishes.

### 4.4 Trackpad Surface Material

Make sure to attach some kind of material to the trackpad surface.
The trackpad will not function properly if used without any material attached.
Typically, we recommend a film thickness of 1-2 mm.

### 4.5 TP Configuration Examples

You can modify the sensor behavior by editing the `src/iqs7211e_init.h` file provided by Azoteq. This file contains all necessary initialization and gesture settings.
Edit values here to adjust:

- Gesture timing, thresholds, and distances
- Report rates and timeouts
- Hardware and ALP settings
- Channel allocation and cycles

For the current datasheet and design references, use Azoteq's official
[IQS7211E product page](https://www.azoteq.com/product/iqs7211e/) and
[application-notes index](https://www.azoteq.com/design/application-notes/).

## 5. Coordinate Pipeline

This section is maintenance material for developers and agents who change the
implementation. It records how the driver treats coordinates, what each setting
trades against what, and how to verify a profile for a given panel.

### 5.1 Processing Stages

One report is processed in this order:

1. **One 12-byte read** — Gesture through Finger 1 Area are fixed by a single I2C
   transfer. Reading them separately would need two communication windows for
   one event, and one of them could be missed.
2. **Contact consistency** — `fingers > 0`, X/Y other than `0xFFFF`, and non-zero
   strength and area are checked together.
3. **First contact** — A contact never starts from invalid coordinates. Once a
   contact is established, a transient invalid frame keeps the previous value
   instead of ending the contact.
4. **Rubber-band deadband** — The output follows the input from deadband pixels
   behind.
5. **Three-sample median** — Removes single outliers.

Absolute and relative modes share stages 2-5. They branch into absolute
reporting and relative deltas only after the common filtered X/Y has been
produced. Moving that boundary forward gives the two modes different coordinate
paths for the same finger movement, so confirm equivalence with
`test_absolute_relative_parity` whenever it changes.

### 5.2 Parameters and State

The coordinate filter's tuning value is exposed as a build-time Device Tree
property.

| Effective setting | Default | Device Tree property that overrides it |
|---|---:|---|
| Rubber-band width | 8 | `jitter-deadband` |

Keep the binding default, the type in `struct iqs7211e_config`, and the
`DT_INST_PROP_OR` fallback identical. The driver default is
`jitter-deadband = 8`. Measure it against the hardware — panel, electrodes and
surface material — and override it in the board overlay where needed.

Mutable state such as the deadband and median history lives in
`struct iqs7211e_data` and is never mixed into the read-only device config.

`touch-verify-interval-ms` is a sensor liveness check that runs in both absolute
and relative modes. It does not depend on the coordinate filter or on layers,
and releases a stale contact when it detects a physical release or a read
failure.

### 5.3 Why Each Stage Exists

#### Why no host-side IIR is stacked

The IQS7211E runs MAV and a dynamic IIR on the chip (datasheet 7.8). Stacking
another IIR on the host doubles the smoothing, which shrinks the movement path
and adds lag. This driver treats the on-chip smoothing as the final result and
has no host-side IIR.

#### Why the deadband is kept minimal

The rubber-band deadband absorbs small tremors at rest and in motion, but a
larger value adds the same per-axis lag to intended movement. Use the smallest
value that suppresses the measured resting noise, balanced against tracking. An
extra gate on the TP Movement flag is not used: measurements showed no
meaningful improvement, and raising the number of confirming reports increased
latency and catch-up jumps.

### 5.4 Verifying Fixed Panel Values

#### Touch SET / CLEAR

The threshold is `Threshold = Reference × (1 + Multiplier / 128)`
(datasheet 5.5.1). A larger multiplier makes the panel less sensitive.

The basic procedure from AZD123 4.3.1 and AZD128 5.5.5 is:

1. Lightly press **between four channels** with a small finger, at a point
   where the four deltas are about equal.
2. Put SET below the smallest of the four channels.
3. Put CLEAR below SET to create hysteresis.

If a hover reaches the same level as a light touch, this procedure and clearing
false detections can conflict: clearing a false detection needs CLEAR above the
hover level, but SET has to stay above CLEAR. When that conflict occurs, record
which side was favoured and why.

#### jitter-deadband

The lower bound is the value that absorbs the residual displacement observed in
a resting log. The upper bound is the value that does not swallow the smallest
intentional movement. Replay a resting log and a small-circle log with the same
setting, and decide by comparing movement at rest, path length in motion,
maximum step and tracking lag.

#### ATI

Following AZD123 4.2.1 and AZD128 5.5.4, confirm that:

- ATI Compensation sits near the middle of 0-1023
- the ATI Error flag (INFO_FLAGS bit 3) is not set
- the reference is within `ATI target ± Reference drift limit`
- the delta on contact is sufficient for the application

Normally leave the coarse divider / multiplier at index 0 of AZD123 table 4.1 and
tune with the fine divider. Do not lower the fine divider below 16.

#### X/Y Trim

AZD128 6.4 requires that both axes reach coordinate 0 and the maximum
resolution. Check the four corners and the four edges separately. A trim acts by
the same amount on both ends of one axis, so an asymmetric excess at only one
end cannot be removed with it.

### 5.5 Checks After a Change

1. Touch the centre lightly and hold still: one contact, zero releases in
   between, and almost no output movement.
2. Tap lightly and repeatedly: the numbers of contacts and releases match.
3. Slow straight lines, circles and fast back-and-forth strokes: zero
   intermediate releases, no missing IRQ/work/report, and zero I2C errors.
4. `tests/filter/run.sh` passes, including the absolute/relative parity test.
5. Compare the final firmware's FLASH/RAM with the previous build.

### 5.6 References

- [IQS7211E product page](https://www.azoteq.com/product/iqs7211e/): datasheet, AZD123
- [Application notes index](https://www.azoteq.com/design/application-notes/): AZD128
