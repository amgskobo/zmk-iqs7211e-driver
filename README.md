# zmk-iqs7211e-driver

[[日本語]](README_JA.md)

<img src=/img/iqs7211e_trackpad01.png width="500px" />

## 1. Overview

This repository provides a driver for the **"Trackpad01"** (Azoteq IQS7211E touch/proximity sensor chip) for ZMK (Zephyr Mechanical Keyboard firmware). It has been verified with **Zephyr 4.1**.
The driver is inspired by the [ZMK PMW3610 driver](https://github.com/inorichi/zmk-pmw3610-driver). While the IQS7211E chip itself supports full 2 fingers input, this small trackpad module **(padsize is 22mmX22mm)** only supports single-finger gestures. Supports standard ZMK interrupt-driven input, enabling responsive event handling.

The driver also implements touch gesture and scroll slider features:

- Single-tap / Double-tap / Triple-tap
- Scroll slider (right-edge area)
  - Activates a specified layer while touching (`scroll-layer = <1>` is generally used)
  - Releases to off the layer
- Precise rotation correction (`rotate-cw`) for flexible physical placement, including automatic slider area adjustment. **Eliminates the need for manual rotation/flipping in input processors.**
- "Ultimate Quality" Rigor: Implemented mathematical boundary fixes (Off-by-one) and safe PM (Power Management) execution guards.

## 2. Device Tree Properties

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `reg` | byte | 0x56 | I2C address of the device (required) |
| `irq-gpios` | phandle-array | | Interrupt pin (required)|
| `single-tap` | int | -1 | Button triggered by single-tap (-1=disabled, 0=BTN_0, 1=BTN_1, 2=BTN_2, ...) |
| `double-tap` | int | -1 | Button triggered by double-tap (-1=disabled, 0=BTN_0, 1=BTN_1, 2=BTN_2, ...) |
| `triple-tap` | int | -1 | Button triggered by triple-tap (-1=disabled, 0=BTN_0, 1=BTN_1, 2=BTN_2, ...) |
| `scroll-layer` | int | -1 | Layer activated while first touching scroll slider area (-1=disabled, others=layer num) |
| `scroll-start` | uint | 40 | Threshold/padding from right edge to activate scroll slider (resolution 0-1024 inclusive) |
| `scroll-trigger-layers` | array | any | Highest active layers that may activate the scroll layer. If omitted, any layer may trigger it. |
| `rotate-cw` | uint | 0 | **CW Rotation angle to match physical placement** (0=0°, 1=90°, 2=180°, 3=270°). Coordinates and scroll area are normalized internally. |
| `report-abs` | boolean | false | If true, report absolute coordinates instead of relative ones. |
| `jitter-deadband` | int | 8 | Per-axis coordinate distance held by the rubber-band jitter gate. X and Y are gated independently, not by Euclidean distance. 0 disables the gate and keeps the three-sample median filter. The default is a conservative Trackpad01 starting point; override it after measuring another panel. Valid range: 0-1024. |
| `stationary-report-interval-ms` | int | 0 | Absolute mode only. With `report-abs`, resend the last absolute coordinate report at this interval while touch remains active. Ignored in relative mode; set to 0 to disable. |
| `stationary-report-layers` | array | any | Absolute mode only. Layers on which stationary absolute resends are allowed. Any listed layer being active is enough, even with another layer above it. If omitted, resends are allowed on any layer. This does not gate touch verification. |
| `touch-verify-interval-ms` | int | 120 | In both report modes, independently run a full report read at this interval while touch remains active. This keeps release recovery and stationary velocity decay equal between absolute and relative reporting. It does not require stationary resends and is not layer-gated. Set to 0 to use the sensor's 60-second fallback instead. |

### 2.1 Absolute Pointer Report Mode

By default, this driver reports relative coordinates (`INPUT_REL_X`, `INPUT_REL_Y`). By setting `report-abs;` in the Device Tree, it will switch to absolute coordinates (`INPUT_ABS_X`, `INPUT_ABS_Y`).
This is useful when combined with ZMK input processors that expect absolute data, such as a digitizer-to-mouse converter.
The absolute coordinates are reported in the range of 0 to 1024 (as defined by the chip's resolution).

Both absolute and relative reporting use the same stateful coordinate filter: a configurable
rubber-band deadband followed by a three-sample median. The driver relies on the sensor's
on-chip MAV and Dynamic IIR rather than applying a second IIR on the host. All filter state is
reset at the start of every contact, and a temporarily invalid coordinate holds the previous
output without advancing the filter. The fixed 22 mm Trackpad01 default is deadband 8; a board
only needs an override when its measurements call for it.

### 2.2 Stationary Absolute Resend and Touch Verification

Stationary resend is available only in absolute-report mode. It requires `report-abs;` and a
non-zero `stationary-report-interval-ms`. Relative mode ignores the resend interval and resend
layers, but touch verification remains active in both modes.

In IQS7211E Event Mode, the sensor may stop generating new events while a finger is held still. If `report-abs` feeds a joystick-style or padstick-style input processor, this can make the downstream processor stop moving even though touch is still active.

`stationary-report-interval-ms` keeps that pipeline alive by periodically resending the last `INPUT_ABS_X` / `INPUT_ABS_Y` report while touch remains active. The feature is disabled by default because it only applies to absolute-report workflows.

`stationary-report-layers` limits the resend to specific layers. This is useful when absolute coordinates feed different processors on different layers. For example, a padstick layer can receive stationary resends, while scroll or matrix layers can avoid them.

The array contains layer numbers, not a bit mask: `<1>` means layer 1 only, and `<0 1>` means layers 0 and 1. This setting controls only periodic resends. It does not enable or change the coordinate filter on those layers.

A resend is allowed whenever any listed layer is active, whether or not another layer sits above it - deliberately not the highest-active-layer test `scroll-trigger-layers` uses. ZMK chooses a processor chain per event from the layer active at that moment, and by the first listener entry that matches rather than by the highest layer, so a listed layer can be the one holding the chain while a higher layer sits above it. Asking only about the top would withhold the resends that chain relies on and stop a stationary contact dead.

`touch-verify-interval-ms` independently checks that the physical touch is still there in both report modes. It does not depend on `stationary-report-interval-ms` and is not gated by `stationary-report-layers`, because resend routing belongs to downstream processors while touch liveness belongs to the sensor driver. Giving both modes the same verify samples also makes stationary velocity decay and release-time inertia agree. If the read fails or the sensor reports no fingers, the driver releases touch and stops any stale coordinate resend. The default is 120 ms; set it to 0 only if the sensor's slower fallback is preferred.

The check is a full report read, not a bare `INFO_FLAGS` poll. A partial read followed by a STOP would close a communication window that had been opened for a gesture or coordinate event and lose it, so the verify goes through the normal report path with the interrupt masked. A verify tick can therefore also emit coordinates and dispatch clicks, exactly as an ordinary report does.

When host verification is active, the driver programs the chip's Idle-Touch timeout to 0 so the chip cannot reseed underneath a touch the host still owns. When the verify interval is 0, the chip's 60-second timeout remains enabled as the stuck-touch fallback in either report mode.

Example:

```dts
report-abs;
stationary-report-interval-ms = <20>;
stationary-report-layers = <1>;
touch-verify-interval-ms = <120>;
```

In this example, stationary resends run every 20 ms whenever layer 1 is active. Touch presence is verified every 120 ms on every layer, including while layer 1 is inactive.

### 2.3 Scroll Layer Trigger Control

`scroll-layer` selects the layer that is activated while the scroll slider area is touched. `scroll-trigger-layers` limits where that automatic activation is allowed.

The driver checks the highest active ZMK layer when a touch starts near the scroll slider area. If that layer is listed in `scroll-trigger-layers`, the driver activates `scroll-layer`. If the current highest active layer is not listed, the touch is handled normally and the scroll layer is not activated.

Example:

```dts
scroll-layer = <6>;
scroll-start = <50>;
scroll-trigger-layers = <0>;
```

In this example, layer 6 is used as the scroll layer, but it can only be triggered from layer 0. This is useful when another layer, such as a padstick or mouse-only layer, should keep the full pad area available without the right edge entering the scroll layer.

If `scroll-trigger-layers` is omitted, the driver keeps the previous behavior and allows any layer to trigger `scroll-layer`.

### 2.4 Filter Tests

The standalone filter tests cover deadband behavior, spike rejection, contact reset, invalid
frames, stationary velocity decay, and absolute/relative parity. The parity scenario derives
each expected relative delta independently from the absolute filtered coordinate stream:

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

        /* Scroll slider settings */
        scroll-layer = <1>;
        scroll-start = <27>;
        // scroll-trigger-layers = <0>; // optional: only these highest active layers may enter scroll mode
        rotate-cw = <0>;
        // report-abs; // Use absolute coordinates (0-1024 inclusive)
        // The following absolute-only options require report-abs.
        // stationary-report-interval-ms = <20>; // optional: resend stationary ABS reports
        // stationary-report-layers = <1>; // optional: resend only while one of these layers is active
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
reports, generated click edges, stationary reports, touch verification, and
suspend-time releases all run there. This is required for reliable input
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
tap sequences, stationary touch verification, and suspend/resume before
reducing the stack size. The diagnostic option is disabled by default.

### 3.4 Build Firmware

Push your changes to your GitHub repository.
The GitHub Actions workflow automatically builds the firmware and generates artifacts (`.uf2` or `.bin`) ready for download.

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

For more details, looking at the datasheet&references:

- [iqs7211e_datasheet](/docs/iqs7211e_datasheet.pdf)
- [azd123_iqs721xy_trackpad_userguide](/docs/azd123_iqs721xy_trackpad_userguide.pdf)
- [azd128-gamepad-trackpad-design-guide_v1.0](/docs/azd128-gamepad-trackpad-design-guide_v1.0.pdf)
