# Using the IQS7211E driver with ZMK Studio

The IQS7211E driver is compatible with ZMK Studio. ZMK Studio changes keymap
bindings at runtime; it does not change the trackpad's DeviceTree properties or
input-processor configuration. Keep those hardware and routing settings in the
keyboard configuration, then use Studio to edit ordinary key bindings.

## 1. Reserve the trackpad's layers in the stock keymap

Every layer named by `scroll-slider-layer`, `scroll-slider-trigger-layers`,
`scroll-layers`, or the matching `zmk,input-listener` entries must exist in the
stock keymap. The driver stores and uses their **layer IDs**. Layer IDs are
stable when Studio reorders layers, unlike their visual positions in Studio.

For the right-edge slider, make the target an exclusive layer. No key binding,
macro, or other behavior may activate it: the driver owns its activation while
a slider contact is in progress. Do not place `&studio_unlock` on this layer.
Put that binding on a normal key instead.

For example, a keymap with an ordinary base layer, a normal function layer,
and a dedicated scroll layer can use IDs 0, 1, and 2:

```dts
/ {
    keymap {
        compatible = "zmk,keymap";

        base { display-name = "Base"; bindings = < /* ... */ >; };
        fn { display-name = "Fn"; bindings = < /* ... */ >; };

        /* The driver activates this layer only while the right edge is touched. */
        scroll { display-name = "Scroll"; bindings = < /* ... */ >; };
    };
};
```

The corresponding IQS7211E node and input listener use the same layer ID:

```dts
&iqs7211e {
    scroll-slider-layer = <2>;
    scroll-slider-trigger-layers = <0 1>;
};

/ {
    trackpad_input_listener {
        scroller {
            layers = <2>;
            input-processors = <&zip_xy_scaler 1 20>,
                               <&zip_xy_to_scroll_mapper>;
        };
    };
};
```

If the user moves `Scroll` in the Studio interface, it retains ID 2; the
driver's trigger check continues to use that ID. The driver converts ZMK's
current highest **layer index** back to its layer ID before testing
`scroll-slider-trigger-layers`.

Reserved Studio-only layers are useful for ordinary keymap expansion, but are
not a substitute for a hardware routing layer. Define a real, named scroll
layer in the stock keymap so the driver and input listener can reference it.

## 2. Enable ZMK Studio for the central side

In the GitHub Actions `build.yaml`, add the USB RPC snippet and Studio Kconfig
setting to the central side only. For a split keyboard, do not add them to the
peripheral build.

```yaml
---
include:
  - board: nice_nano//zmk
    shield: your_keyboard_left
    snippet: studio-rpc-usb-uart
    cmake-args: -DCONFIG_ZMK_STUDIO=y
  - board: nice_nano//zmk
    shield: your_keyboard_right
```

Add a `&studio_unlock` binding to an ordinary key in the keymap. Studio cannot
edit the driver DeviceTree properties, input-processor chains, or define new
behaviors; change those in the configuration and rebuild the firmware.

For a local central-side build, the equivalent is:

```sh
west build -d build/studio -b nice_nano//zmk \
  -S studio-rpc-usb-uart -- -DSHIELD=your_keyboard_left -DCONFIG_ZMK_STUDIO=y
```

## 3. First-flash and operating checks

1. Flash the Studio-enabled firmware, connect over the same selected USB or
   BLE output endpoint, and press the key containing `&studio_unlock`.
2. Confirm the base, function, and dedicated scroll layers appear in Studio.
3. Reorder a normal layer in Studio, then check that the right-edge slider is
   enabled only from the configured trigger layers and that a normal tap still
   produces a click.
4. Confirm the scroll layer is released after lifting a finger, after a sensor
   reset, and after suspend/resume.

After Studio saves a keymap, later edits to the compiled `.keymap` file are not
applied until **Restore Stock Settings** is selected in Studio. Keep a backup
of any Studio-edited keymap before restoring stock settings.

See the official [ZMK Studio documentation](https://zmk.dev/docs/features/studio)
for board physical-layout requirements, transport setup, and memory limits.
