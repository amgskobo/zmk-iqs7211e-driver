# Using the IQS7211E driver with ZMK Studio

The IQS7211E driver is compatible with ZMK Studio. ZMK Studio changes keymap
bindings at runtime; it does not change the trackpad's DeviceTree properties or
input-processor configuration. Keep those hardware and routing settings in the
keyboard configuration, then use Studio to edit ordinary key bindings.

## 1. Keep routing outside the sensor driver

The IQS7211E driver does not reference keymap layers. Define sliders and other
temporary routing with an input processor such as
[`zmk-input-temp-layer-touch`](https://github.com/amgskobo/zmk-input-temp-layer-touch).
Every target and trigger layer used by that processor must exist in the stock
keymap. Its custom-settings integration can expose the processor's enabled,
layer and width values to a compatible Studio client without coupling those
settings to this sensor driver.

## 2. Enable ZMK Studio for the central side

In the GitHub Actions `build.yaml`, add the USB RPC snippet and Studio Kconfig
setting to the central side only. For a split keyboard, do not add them to the
peripheral build.

```yaml
---
include:
  - board: <central-board>
    shield: your_keyboard_left
    snippet: studio-rpc-usb-uart
    cmake-args: -DCONFIG_ZMK_STUDIO=y
  - board: <peripheral-board>
    shield: your_keyboard_right
```

Add a `&studio_unlock` binding to an ordinary key in the keymap. Studio cannot
edit the driver DeviceTree properties, input-processor chains, or define new
behaviors; change those in the configuration and rebuild the firmware.

For a local central-side build, the equivalent is:

```sh
west build -d build/studio -b <central-board> \
  -S studio-rpc-usb-uart -- -DSHIELD=your_keyboard_left -DCONFIG_ZMK_STUDIO=y
```

## 3. First-flash and operating checks

1. Flash the Studio-enabled firmware, connect over the same selected USB or
   BLE output endpoint, and press the key containing `&studio_unlock`.
2. Confirm every layer referenced by downstream input processors appears in
   Studio.
3. Reorder a normal layer in Studio, then check every configured edge route and
   verify that a normal tap still produces a click.
4. Confirm each temporary layer is released after lifting a finger, after a
   sensor reset, and after suspend/resume.

After Studio saves a keymap, later edits to the compiled `.keymap` file are not
applied until **Restore Stock Settings** is selected in Studio. Keep a backup
of any Studio-edited keymap before restoring stock settings.

See the official [ZMK Studio documentation](https://zmk.dev/docs/features/studio)
for board physical-layout requirements, transport setup, and memory limits.
