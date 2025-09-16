# zmk-iqs7211e-driver
<img src=/img/iqs7211e_trackpad01.png width="30%" />

## 1. Overview
This repository provides a driver for the **Azoteq IQS7211E touch/proximity** sensor for ZMK (Zephyr Mechanical Keyboard firmware) 3.5.
The driver is inspired by the [ZMK PMW3610 driver](https://github.com/inorichi/zmk-pmw3610-driver). While the IQS7211E chip itself supports full 2 fingers input, this small trackpad module only supports single-finger gestures. Supports standard ZMK interrupt-driven input, enabling responsive event handling.

The driver also implements touch gesture and scroll slider features:
- Single-tap / Double-tap / Triple-tap
- Tap & Hold
- Scroll slider (right-edge area)
  - Activates a specified layer while touching (`scroll_layer=1` is generally used)
  - Releases to off the temp layer
- Rotation correction for flexible physical placement

## 2. Device Tree Properties

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `reg` | int | required | I2C address of the device |
| `irq-gpios` | phandle-array | required | Interrupt pin |
| `single-tap` | int | 0 | Button triggered by single-tap (0=disabled, 1=BTN_0, 2=BTN_1, 3=BTN_2) |
| `double-tap` | int | 0 | Button triggered by double-tap (0=disabled, 1=BTN_0, 2=BTN_1, 3=BTN_2) |
| `triple-tap` | int | 0 | Button triggered by triple-tap (0=disabled, 1=BTN_0, 2=BTN_1, 3=BTN_2) |
| `press-hold` | int | 0 | Button triggered by tap-and-hold (0=disabled, 1=BTN_0, 2=BTN_1, 3=BTN_2)|
| `scroll_layer` | int | 0 | Layer activated while first touching scroll slider area (0=disabled, others=layer num) |
| `scroll_start` | int | 20 | Threshold/padding from right edge to activate scroll slider (max resolution 1024x/1024y) |
| `rotate_cw` | int | 0 | Rotation angle for scroll slider area **Clockwise** (0=0°, 1=90°, 2=180°, 3=270°) |


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
        single-tap = <1>;
        double-tap = <1>;
        triple-tap = <1>;
        press-hold = <1>;

        /* Scroll slider settings */
        scroll_layer = <1>;
        scroll_start = <20>;
        rotate_cw = <0>;
    };
};

/ {
   trackpad_input_listener: trackpad_input_listener {
        compatible = "zmk,input-listener";
        status = "okay";
        device = <&iqs7211e>;
        input-processors = <&zip_xy_scaler 4 5>;
        /* Scroll slider settings with using custom input-processors */
        scroller {
            layers = <1>;
            input-processors = <&zip_xy_scaler 1 30>, 
                               <&zip_xy_transform (INPUT_TRANSFORM_Y_INVERT)>,
                               <&zip_xy_to_scroll_mapper>;
        };
    };
};
```

### 3.3 Optional: Enable Driver in Kconfig

If required, add the driver to your `board.conf`:

```
CONFIG_I2C=y
CONFIG_GPIO=y
CONFIG_INPUT=y
CONFIG_ZMK_POINTING=y
CONFIG_ZMK_IQS7211E=y
```

### 3.4 Build Firmware

Push your changes to your GitHub repository.  
The GitHub Actions workflow automatically builds the firmware and generates artifacts (`.uf2` or `.bin`) ready for download.

## 4. HW and Dimensions

### 4.1 Front view
<img src=/img/iqs7211e_trackpad01_front.png width="50%" />

### 4.2 Back view

<img src=/img/iqs7211e_trackpad01_back.png width="50%" />

### 4.3 Pinout (all +3V3 logic) 
| PIN | value | info |
|-----|-------|------|
|1  |  GND |  - |
|2  |  GND |  - |
|3  |  RDY | interrupt pin |
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
| `J1` | PinHeader_2x03_P2.54mm_Vertical | 2x3pin 2.54mm pitch 3.5mm height| 1 | [aliexpress](https://ja.aliexpress.com/item/1005003263426999.html?spm=a2g0o.order_list.order_list_main.16.5d86585aR1YHtk&gatewayAdapt=glo2jpn) |
| `U1` | IQS7211E001QNR |  iqs7211E| 1| [digikey](https://www.digikey.jp/en/products/detail/azoteq-pty-ltd/IQS7211E001QNR/18627341)|

### 4.5 PCBA layout
The PCB used with this driver is a 2-layer FR4 board with a standard thickness of 1.6 mm. The recommended finish for the PCB is ENIG (Electroless Nickel Immersion Gold).

The ENIG finish provides high durability for the edges of the trackpad and connector areas, allowing for long-term stable use. In addition, the gold layer prevents oxidation, ensuring stable touch sensitivity and response.

Please note that if the PCB thickness is different from 1.6 mm, it may affect the installation and feel of the trackpad. Also, the ENIG finish may incur higher costs compared to standard finishes.

### 4.5 Trackpad Surface Material
Make sure to attach some kind of material to the trackpad surface.
The trackpad will not function properly if used without any material attached. 
Typically, we recommend a film thickness of 1-2 mm.

### 4.6 TP Configuration Examples

You can modify the sensor behavior by editing the `IQS7211E_init.h` file provided by Azoteq. This file contains all necessary initialization and gesture settings.
Edit values here to adjust:
- Gesture timing, thresholds, and distances
- Report rates and timeouts
- Hardware and ALP settings
- Channel allocation and cycles

For more details, looking at the datasheet&references:
- [iqs7211e_datasheet](/docs/iqs7211e_datasheet.pdf)
- [azd123_iqs721xy_trackpad_userguide](/docs/azd123_iqs721xy_trackpad_userguide.pdf)
- [azd128-gamepad-trackpad-design-guide_v1.0](/docs/azd128-gamepad-trackpad-design-guide_v1.0.pdf)
