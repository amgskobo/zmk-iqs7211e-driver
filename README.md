# zmk-iqs7211e-driver

[[日本語]](README_JA.md)

<img src=/img/iqs7211e_trackpad01.png width="500px" />

## 1. Overview

This repository provides a driver for the **"Trackpad01"** (Azoteq IQS7211E touch/proximity sensor chip) for ZMK (Zephyr Mechanical Keyboard firmware). It has been verified with **Zephyr 4.1**.
The driver is inspired by the [ZMK PMW3610 driver](https://github.com/inorichi/zmk-pmw3610-driver). While the IQS7211E chip itself supports full 2 fingers input, this small trackpad module **(padsize is 22mmX22mm)** only supports single-finger gestures. Supports standard ZMK interrupt-driven input, enabling responsive event handling.

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
| `jitter-deadband` | int | 8 | Per-axis coordinate distance held by the rubber-band jitter gate. X and Y are gated independently, not by Euclidean distance. 0 disables the gate and keeps the three-sample median filter. The default is a conservative Trackpad01 starting point; override it after measuring another panel. Valid range: 0-1024. |
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
output without advancing the filter. The fixed 22 mm Trackpad01 default is deadband 8; a board
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
| 5 | Noise boundary | `jitter-deadband` | Measure from the Trackpad01 default `<8>`. Raise it for stationary jitter, lower it if fine movement is lost. `0` disables only the deadband; the median filter remains. |
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
sh tests/filter/run.sh
```

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

Add the IQS7211E node in your keyboard DTS overlay file (example of XIAO_BLE board):

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

### 4.1 Trackpad01 Front view (HASL)

<img src=/img/iqs7211e_trackpad01_front.png width="500px" />

### 4.2 Trackpad01 Back view (HASL)

<img src=/img/iqs7211e_trackpad01_back.png width="500px" />

### 4.3 Pin Assignment (all +3V3 logic)

| PIN | value | info |
|-----|-------|------|
|1  |  GND |  - |
|2  |  GND |  - |
|3  |  RDY | irq interrupt pin |
|4  |  +3V3 | VDD |
|5  |  SDA | i2c data|
|6  |  SCL | i2c clock |

### 4.4 BOMs

| Property | Value | Type | Qty | Link |
|----------|------|---------|-------------|-----|
| `C1,C3,C5` | 100pF | 0805_SMD | 3 | |
| `C2,C4` | 2.2uF | 0805_SMD | 2 | |
| `C6` | 4.7uF | 0805_SMD | 1 | |
| `C7` | 100nF | 0805_SMD | 1 | |
| `R1,R2,R3` | 4.7k | 0805_SMD | 3| |
| `J1` | PinHeader_2x03_P2.54mm_Vertical | 2x3pin 2.54mm pitch PH3.5mm height| 1 | [aliexpress](https://ja.aliexpress.com/item/1005003263426999.html?spm=a2g0o.order_list.order_list_main.16.5d86585aR1YHtk&gatewayAdapt=glo2jpn) |
| `U1` | IQS7211E001QNR |  IQS7211E001QNR(20-QFN)| 1| [digikey](https://www.digikey.jp/en/products/detail/azoteq-pty-ltd/IQS7211E001QNR/18627341)|

### 4.5 PCB Specifications

The PCB used with this driver is a 2-layer FR4 board with a standard thickness of 1.6 mm. The recommended finish for the PCB is ENIG (Electroless Nickel Immersion Gold).

The ENIG finish provides high durability for the edges of the trackpad and connector areas, allowing for long-term stable use. In addition, the gold layer prevents oxidation, ensuring stable touch sensitivity and response.

Please note that if the PCB thickness is different from 1.6 mm, it may affect the installation and feel of the trackpad. Also, the ENIG finish may incur higher costs compared to standard finishes.

### 4.6 Trackpad Surface Material

Make sure to attach some kind of material to the trackpad surface.
The trackpad will not function properly if used without any material attached.
Typically, we recommend a film thickness of 1-2 mm.

### 4.7 TP Configuration Examples

You can modify the sensor behavior by editing the `src/iqs7211e_init.h` file provided by Azoteq. This file contains all necessary initialization and gesture settings.
Edit values here to adjust:

- Gesture timing, thresholds, and distances
- Report rates and timeouts
- Hardware and ALP settings
- Channel allocation and cycles

For the current datasheet and design references, use Azoteq's official
[IQS7211E product page](https://www.azoteq.com/product/iqs7211e/) and
[application-notes index](https://www.azoteq.com/design/application-notes/).
