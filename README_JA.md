# zmk-iqs7211e-driver (日本語)

[[English]](README.md)

<img src=/img/iqs7211e_trackpad01.png width="500px" />

## 1. 概要

このリポジトリは、**"Trackpad01"** (Azoteq IQS7211E タッチ/プロキシミティセンサチップ) 用の ZMK (Zephyr Mechanical Keyboard) ドライバーを提供します。**Zephyr 4.1** で動作確認済みです。

このドライバーは [ZMK PMW3610 driver](https://github.com/inorichi/zmk-pmw3610-driver) から着想を得ています。IQS7211E チップ自体は 2 本指入力をフルサポートしていますが、この小型トラックパッドモジュール **(パッドサイズ 22mm x 22mm)** は、シングルフィンガージェスチャーのみをサポートします。標準的な ZMK の割り込み駆動入力をサポートし、レスポンスの良いイベント処理を実現しています。

また、以下のタッチジェスチャーとスクロールスライダー機能を実装しています：

- シングルタップ / ダブルタップ / トリプルタップ
- タップ & ホールド
- スクロールスライダー (右端エリア)
  - タッチ中に指定したレイヤーをアクティブにします (`scroll_layer = <1>` が一般的です)
  - 離すとレイヤーをオフにします
- 時計回りの回転補正 (`rotate_cw`): スクロールエリアの自動追従を含む、高精度な座標変換。**インプットプロセッサー側での座標変換（回転・反転）を不要にします。**
- 「最奥」クオリティの堅牢性: 境界値の数学的厳密化 (Off-by-one Fix) および PM (電源管理) 時の安全な通信停止を実装。

## 2. デバイスツリーのプロパティ

| プロパティ | 型 | デフォルト | 説明 |
|----------|------|---------|-------------|
| `reg` | byte | 0x56 | デバイスの I2C アドレス (必須) |
| `irq-gpios` | phandle-array | | 割り込みピン (必須) |
| `single-tap` | int | -1 | シングルタップでトリガーされるボタン (-1=無効, 0=BTN_0, 1=BTN_1, ...) |
| `double-tap` | int | -1 | ダブルタップでトリガーされるボタン (-1=無効, 0=BTN_0, 1=BTN_1, ...) |
| `triple-tap` | int | -1 | トリプルタップでトリガーされるボタン (-1=無効, 0=BTN_0, 1=BTN_1, ...) |
| `press-hold` | int | -1 | タップ & ホールドでトリガーされるボタン (-1=無効, 0=BTN_0, 1=BTN_1, ...)|
| `scroll_layer` | int | -1 | スクロールスライダーエリアをタッチした時にアクティブになるレイヤー (-1=無効, その他=レイヤー番号) |
| `scroll_start` | uint | 40 | スクロールスライダーを有効にする右端からの閾値/パディング (最大解像度 1024x/1024y) |
| `rotate_cw` | uint | 0 | **物理配置に合わせた時計回りの回転角度** (0=0°, 1=90°, 2=180°, 3=270°)。ドライバ内部でスクロールエリア判定も含めて一括して座標変換を行います。 |
| `report-abs` | boolean | false | true の場合、相対座標ではなく絶対座標を報告します。 |

### 2.1 絶対座標レポートモード

デフォルトでは、このドライバーは相対座標 (`INPUT_REL_X`, `INPUT_REL_Y`) を報告します。デバイスツリーで `report-abs;` を設定すると、絶対座標 (`INPUT_ABS_X`, `INPUT_ABS_Y`) に切り替わります。
これは、デジタイザーからマウスへの変換器など、絶対データを期待する ZMK 入力プロセッサと組み合わせる場合に便利です。
絶対座標は 0 から 1024 の範囲で報告されます (チップの解像度定義による)。

## 3. インストール (GitHub Actions)

> **注:** ここでは GitHub Actions を使用したビルドのみを扱います。ローカルビルドはユーザー環境によって異なるため対象外です。

### 3.1 `west` マニフェストによるドライバーの追加

ZMK リポジトリの `config/west.yml` にこのドライバーを含めます：

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

これにより、ビルド時に GitHub Actions が自動的に **IQS7211E ドライバー**をプルします。

### 3.2 デバイスツリーオーバーレイの設定

キーボードの DTS オーバーレイファイルに IQS7211E ノードを追加します (XIAO_BLE ボードの例)：

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

        /* タップジェスチャー設定 */
        single-tap = <0>;
        double-tap = <0>;
        triple-tap = <0>;
        // press-hold = <0>;  // 使用しない場合

        /* スクロールスライダー設定 */
        scroll_layer = <1>;
        scroll_start = <27>;
        rotate_cw = <0>;
        // report-abs; // 絶対座標を使用する場合 (0-1024)
    };
};

/ {
    trackpad_input_listener: trackpad_input_listener {
        compatible = "zmk,input-listener";
        status = "okay";
        device = <&iqs7211e>;
        /* カスタム入力プロセッサを使用したスクロールスライダー設定 */
        scroller {
            layers = <1>;
            input-processors = <&zip_xy_scaler 1 20>, 
                               <&zip_xy_transform (INPUT_TRANSFORM_Y_INVERT)>,
                               <&zip_xy_to_scroll_mapper>;
        };
    };
};
```

出力される座標はドライバ内部ですでに補正済みのため、後続の `trackpad_input_listener` では回転や反転の処理 (`zip_xy_transform`, `zip_xy_swap_mapper` 等) を記述する必要はなく、感度調整や慣性移動（inertia）の適用に専念できます。

オプション：90度回転 (rotate_cw = <1>)

```dts
/ {
    trackpad_input_listener: trackpad_input_listener {
        compatible = "zmk,input-listener";
        status = "okay";
        device = <&iqs7211e>;
        /* ドライバ側で回転済みのため、ここでは軸変換は不要 */
        input-processors = <&zip_xy_scaler 1 1>; 
        scroller {
            layers = <1>;
            input-processors = <&zip_xy_scaler 1 20>, 
                               <&zip_xy_to_scroll_mapper>;
        };
    };
};
```

### 3.3 Kconfig でドライバーを有効にする

`board.conf` にドライバーを追加します：

```kconfig
CONFIG_I2C=y
CONFIG_GPIO=y
CONFIG_INPUT=y
CONFIG_ZMK_POINTING=y
CONFIG_IQS7211E=y
```

### 3.4 ファームウェアのビルド

変更を GitHub リポジトリにプッシュします。
GitHub Actions のワークフローが自動的にファームウェアをビルドし、ダウンロード可能なアーティファクト (`.uf2` または `.bin`) を生成します。

## 4. ハードウェアと寸法

### 4.1 Trackpad01 前面図 (HASL)

<img src=/img/iqs7211e_trackpad01_front.png width="500px" />

### 4.2 Trackpad01 背面図 (HASL)

<img src=/img/iqs7211e_trackpad01_back.png width="500px" />

### 4.3 ピンアサイン (すべて +3.3V ロジック)

| ピン | 値 | 情報 |
|-----|-------|------|
|1  |  GND |  - |
|2  |  GND |  - |
|3  |  RDY | 割り込みピン (IRQ) |
|4  |  +3V3 | VDD |
|5  |  SDA | I2C データ |
|6  |  SCL | I2C クロック |

### 4.4 BOM (部品表)

| 項目 | 値 | タイプ | 数量 | リンク |
|----------|------|---------|-------------|-----|
| `C1,C3,C5` | 100pF | 0805_SMD | 3 | |
| `C2,C4` | 2.2uF | 0805_SMD | 2 | |
| `C6` | 4.7uF | 0805_SMD | 1 | |
| `C7` | 100nF | 0805_SMD | 1 | |
| `R1,R2,R3` | 4.7k | 0805_SMD | 3| |
| `J1` | PinHeader_2x03_P2.54mm_Vertical | 2x3ピン 2.54mmピッチ PH3.5mm高さ| 1 | [aliexpress](https://ja.aliexpress.com/item/1005003263426999.html) |
| `U1` | IQS7211E001QNR |  IQS7211E001QNR(20-QFN)| 1| [digikey](https://www.digikey.jp/en/products/detail/azoteq-pty-ltd/IQS7211E001QNR/18627341)|

### 4.5 PCB 仕様

このドライバーで使用される PCB は、標準の厚さ 1.6 mm の 2 層 FR4 基板です。PCB の表面処理は ENIG (無電解ニッケル置換金めっき) を推奨します。

ENIG 処理は、トラックパッドの端やコネクタ部分に高い耐久性をもたらし、長期間の安定した使用を可能にします。また、金層が酸化を防ぎ、安定したタッチ感度とレスポンスを保証します。

PCB の厚さが 1.6 mm と異なる場合、トラックパッドの取り付けや操作感に影響を与える可能性があることに注意してください。また、ENIG 処理は標準的な表面処理と比較してコストが高くなる場合があります。

### 4.6 トラックパッド表面の素材

トラックパッドの表面には必ず何らかの素材を貼り付けてください。
素材を貼り付けずに使用すると、トラックパッドが正常に機能しません。
通常、1〜2 mm の厚さのフィルムを推奨します。

### 4.7 トラックパッド構成例

Azoteq が提供する `src/iqs7211e_init.h` ファイルを編集することで、センサーの挙動を変更できます。このファイルには、必要なすべての初期化およびジェスチャー設定が含まれています。
以下の調整のために値を編集してください：

- ジェスチャーのタイミング、閾値、距離
- レポートレートとタイムアウト
- ハードウェアおよび ALP 設定
- チャンネル割り当てとサイクル

詳細は以下のデータシートおよびリファレンスを参照してください：

- [iqs7211e_datasheet](/docs/iqs7211e_datasheet.pdf)
- [azd123_iqs721xy_trackpad_userguide](/docs/azd123_iqs721xy_trackpad_userguide.pdf)
- [azd128-gamepad-trackpad-design-guide_v1.0](/docs/azd128-gamepad-trackpad-design-guide_v1.0.pdf)
