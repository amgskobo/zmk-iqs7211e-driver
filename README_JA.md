# zmk-iqs7211e-driver (日本語)

[[English]](README.md)

<img src=/img/iqs7211e_trackpad01.png width="500px" />

## 1. 概要

このリポジトリは、**"Trackpad01"** (Azoteq IQS7211E タッチ/プロキシミティセンサチップ) 用の ZMK (Zephyr Mechanical Keyboard) ドライバーを提供します。**Zephyr 4.1** で動作確認済みです。

このドライバーは [ZMK PMW3610 driver](https://github.com/inorichi/zmk-pmw3610-driver) から着想を得ています。IQS7211E チップ自体は 2 本指入力をフルサポートしていますが、この小型トラックパッドモジュール **(パッドサイズ 22mm x 22mm)** は、シングルフィンガージェスチャーのみをサポートします。標準的な ZMK の割り込み駆動入力をサポートし、レスポンスの良いイベント処理を実現しています。

また、以下のタッチジェスチャーとスクロールスライダー機能を実装しています：

- シングルタップ / ダブルタップ / トリプルタップ
- スクロールスライダー (右端エリア)
  - タッチ中に指定したレイヤーをアクティブにします (`scroll-layer = <1>` が一般的です)
  - 離すとレイヤーをオフにします
- 時計回りの回転補正 (`rotate-cw`): スクロールエリアの自動追従を含む、高精度な座標変換。**インプットプロセッサー側での座標変換（回転・反転）を不要にします。**
- 「最奥」クオリティの堅牢性: 境界値の数学的厳密化 (Off-by-one Fix) および PM (電源管理) 時の安全な通信停止を実装。

## 2. デバイスツリーのプロパティ

| プロパティ | 型 | デフォルト | 説明 |
|----------|------|---------|-------------|
| `reg` | byte | 0x56 | デバイスの I2C アドレス (必須) |
| `irq-gpios` | phandle-array | | 割り込みピン (必須) |
| `single-tap` | int | -1 | シングルタップでトリガーされるボタン (-1=無効, 0=BTN_0, 1=BTN_1, ...) |
| `double-tap` | int | -1 | ダブルタップでトリガーされるボタン (-1=無効, 0=BTN_0, 1=BTN_1, ...) |
| `triple-tap` | int | -1 | トリプルタップでトリガーされるボタン (-1=無効, 0=BTN_0, 1=BTN_1, ...) |
| `scroll-layer` | int | -1 | スクロールスライダーエリアをタッチした時にアクティブになるレイヤー (-1=無効, その他=レイヤー番号) |
| `scroll-start` | uint | 40 | スクロールスライダーを有効にする右端からの閾値/パディング (解像度 0-1024、最大値を含む) |
| `scroll-trigger-layers` | array | any | スクロールレイヤーを自動有効化してよい highest active layer。省略時はどのレイヤーでも有効化します。 |
| `rotate-cw` | uint | 0 | **物理配置に合わせた時計回りの回転角度** (0=0°, 1=90°, 2=180°, 3=270°)。ドライバ内部でスクロールエリア判定も含めて一括して座標変換を行います。 |
| `report-abs` | boolean | false | true の場合、相対座標ではなく絶対座標を報告します。 |
| `jitter-deadband` | int | 8 | ラバーバンド式ジッタゲートが保持する軸ごとの座標差。X/Yへ独立に適用し、ユークリッド距離では判定しません。0ではゲートだけを無効にし、3サンプル中央値フィルターは維持します。既定値はTrackpad01向けの保守的な開始値で、別パネルでは実測後に上書きします。有効範囲は0～1024です。 |
| `stationary-report-interval-ms` | int | 0 | 絶対モード専用です。`report-abs` 有効時、タッチ中に最後の絶対座標レポートをこの周期で再送します。相対モードでは無視され、0で無効です。 |
| `stationary-report-layers` | array | any | 絶対モード専用です。stationary absolute resendを許可するレイヤー。上に別のレイヤーが乗っていても、列挙したいずれかが有効なら再送します。省略時はどのレイヤーでも再送します。touch verifyは制限しません。 |
| `touch-verify-interval-ms` | int | 120 | 絶対・相対の両モードで、タッチ継続中に通常レポートをこの周期で1回読みます。release復旧と静止中の相対速度減衰を両モードで揃えます。stationary resendやレイヤー制限には依存しません。0ではホスト側verifyを無効化し、チップ側の60秒fallbackを使います。 |

### 2.1 絶対座標レポートモード

デフォルトでは、このドライバーは相対座標 (`INPUT_REL_X`, `INPUT_REL_Y`) を報告します。デバイスツリーで `report-abs;` を設定すると、絶対座標 (`INPUT_ABS_X`, `INPUT_ABS_Y`) に切り替わります。
これは、デジタイザーからマウスへの変換器など、絶対データを期待する ZMK 入力プロセッサと組み合わせる場合に便利です。
絶対座標は 0 から 1024 の範囲で報告されます (チップの解像度定義による、最大値を含む)。

絶対モードと相対モードは、設定可能なラバーバンドdeadbandと3サンプル中央値から成る同じ座標フィルターを使います。一時的な無効座標は直前の出力を保持し、フィルター履歴も進めません。平滑化はチップ内MAVとDynamic IIRに任せ、ホスト側でIIRを重ねません。すべてのフィルター状態は接触開始ごとに初期化されます。

フィルター設定はビルド時のDevice Treeプロパティです。bindingが既定値を定義し、選択値を読み取り専用のdevice configへ格納し、接触ごとに変化するフィルター状態はdevice dataへ分離します。固定の22 mm Trackpad01向けdriver既定値は`jitter-deadband = 8`です。絶対・相対座標へ分岐する前に同じ処理を適用し、別の値が必要なボードだけoverlayで上書きします。

### 2.2 静止中の絶対座標再送とタッチ確認

stationary resendは絶対座標モード専用です。`report-abs;`と0以外の`stationary-report-interval-ms`の両方が必要です。相対座標モードでは再送周期と再送レイヤーを無視します。一方、touch verifyは両モードで動作します。

IQS7211E の Event Mode では、指を止めたままにすると新しいイベントが発生しなくなることがあります。`report-abs` を joystick や padstick のような入力プロセッサへ渡す場合、タッチは継続していても下流のプロセッサが止まったように見えることがあります。

`stationary-report-interval-ms` は、タッチ中に最後の `INPUT_ABS_X` / `INPUT_ABS_Y` を定期的に再送して、この入力パイプラインを動かし続けます。この機能は絶対座標レポートのワークフロー向けなので、デフォルトでは無効です。

`stationary-report-layers` で、再送を許可するレイヤーを制限できます。レイヤーごとに絶対座標を別の入力プロセッサへ渡す構成で便利です。たとえば padstick レイヤーでは再送し、scroll や matrix レイヤーでは再送しない、という使い分けができます。

配列の値はビットマスクではなくレイヤー番号です。`<1>`はlayer 1だけ、`<0 1>`はlayer 0と1を表します。この設定が制御するのは周期的な再送だけであり、そのレイヤーの座標フィルターを有効化したり強さを変更したりはしません。

判定は「列挙したいずれかのレイヤーが有効か」で行い、上に別のレイヤーが乗っていても許可します。`scroll-trigger-layers` の highest active layer 判定とは意図的に異なります。ZMK はプロセッサチェーンをイベントごとに、その時点で有効なレイヤーから、かつリスナの**最初に一致した項目**で選びます（最上位レイヤーではありません）。そのため、列挙したレイヤーがチェーンを保持している最中に、より上のレイヤーが乗ることがあります。最上位だけを見ると、そのチェーンが頼っている再送を止めてしまい、静止した接触がそこで死にます。

`touch-verify-interval-ms` は、絶対・相対の両モードで、タッチ継続中に物理タッチが残っているかを独立して確認します。`stationary-report-interval-ms` が0でも動作し、`stationary-report-layers` のレイヤー制限も受けません。再送先の選択はプロセッサ側の都合ですが、タッチの生存確認はセンサードライバーの責務だからです。両モードが同じ確認サンプルを受けるため、release復旧だけでなく、相対モードの静止中速度減衰と指を離した後の慣性も揃います。読み取りに失敗した場合やセンサーが指なしを報告した場合、ドライバーはタッチをreleaseして古い座標の再送も止めます。デフォルトは120 msです。このホスト側fail-safeが不要な場合のみ0にしてください。

この確認は `INFO_FLAGS` だけの読み取りではなく、通常のレポート経路を1回丸ごと通します。部分的に読んで STOP すると、ジェスチャーや座標イベントのために開いていた communication window を閉じてしまい、そのイベントを失うためです。したがって verify のタイミングでも座標の出力やクリックの発行が起こり得ます。

ホスト側verifyが有効な場合、チップがホストの保持中に参照値を再seedしないよう、Idle-Touch timeoutは0に設定します。verify周期0の場合だけ、チップ側の60秒timeoutをstuck-touchのfallbackとして残します。

例:

```dts
report-abs;
stationary-report-interval-ms = <20>;
stationary-report-layers = <1>;
touch-verify-interval-ms = <120>;
```

この例では、layer 1 が有効な間は20 msごとにstationary resendを行います。タッチの存在確認はlayer 1が無効な間も含め、全レイヤーで120 msごとに動作します。

### 2.3 スクロールレイヤーの発火制御

`scroll-layer` は、スクロールスライダーエリアをタッチした時に有効化するレイヤーを指定します。`scroll-trigger-layers` は、その自動有効化をどのレイヤーから許可するかを制限します。

ドライバは、スクロールスライダーエリア付近でタッチが始まった時点の highest active layer を確認します。その layer が `scroll-trigger-layers` に含まれていれば `scroll-layer` を有効化します。含まれていなければ、スクロールレイヤーには入らず通常のタップ/ジェスチャーとして処理します。

例:

```dts
scroll-layer = <6>;
scroll-start = <50>;
scroll-trigger-layers = <0>;
```

この例では layer 6 をスクロール用レイヤーとして使いますが、発火できるのは layer 0 が highest active layer の時だけです。padstick や mouse-only 用のレイヤーでは、右端も含めたパッド全体を使いたい場合に有効です。

`scroll-trigger-layers` を省略した場合は従来どおり、どの layer からでも `scroll-layer` を発火できます。

### 2.4 フィルターテスト

デッドバンド、瞬間的な外れ値、接触開始時の初期化、一時的な無効座標、静止サンプルによる相対速度の減衰、絶対・相対モードの同等性は次のテストで確認できます。相対deltaは、絶対モードと同じfiltered座標から導出されることを確認します。

```sh
sh tests/filter/run.sh
```

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

        /* スクロールスライダー設定 */
        scroll-layer = <1>;
        scroll-start = <27>;
        // scroll-trigger-layers = <0>; // 任意: 指定した highest active layer の時だけ scroll mode に入る
        rotate-cw = <0>;
        // report-abs; // 絶対座標を使用する場合 (0-1024、最大値を含む)
        // 以下の絶対モード専用設定には report-abs が必要です。
        // stationary-report-interval-ms = <20>; // 任意: 静止中の ABS レポートを再送する
        // stationary-report-layers = <1>; // 任意: 列挙したいずれかのレイヤーが有効な間だけ再送する
        // touch-verify-interval-ms = <120>; // 任意: レイヤー非依存でタッチ継続を確認する
    };
};

/ {
    trackpad_input_listener: trackpad_input_listener {
        compatible = "zmk,input-listener";
        status = "okay";
        device = <&iqs7211e>;
        /* ドライバ側で回転補正が行われるため、リスナー側での zip_xy_transform 等は不要 */
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

ドライバーは、すべての IQS7211E instance で1つの専用 work queue を共有します。センサー報告、クリックの press/release、静止中の再送、touch verify、suspend 時の release はすべてこの queue 上で順番に処理します。Zephyr の非同期 input backend は system work queue からの報告を non-blocking に変更するため、input queue が満杯になると release を落とす可能性があります。専用 queue では `K_FOREVER` の待機が有効なままになり、input thread が queue を空けるまで待つため、press/release の順序を保持できます。

RDY 割り込みを再有効化した後は、少し遅らせて logical level も確認します。割り込みを mask している間に RDY が active になっていた場合は、次の edge を待たずに report work を再投入します。高速な復旧回数には上限を設け、その後は間隔を広げるため、RDY pin が異常に active のままでも work queue を占有し続けません。

通常は次の production 既定値のままで使用します。

```kconfig
CONFIG_IQS7211E_WORKQUEUE_STACK_SIZE=1536
CONFIG_IQS7211E_WORKQUEUE_PRIORITY=-1
```

スタックを実測する診断 firmware では `CONFIG_IQS7211E_WORKQUEUE_STACK_USAGE=y` を有効にします。peak 使用量が増えるたびに high-water mark をログへ出します。移動、連続 tap、静止中の touch verify、suspend/resume を一通り実行してから stack size を減らしてください。この診断設定は既定では無効です。

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

座標処理を変更するときは、以下の「座標パイプライン」も参照してください。

## 5. 座標パイプライン

この節は、実装を変更する開発者やエージェント向けの保守資料です。ドライバーが座標をどう扱うか、各設定が何と何を引き換えにしているか、固定の22 mm Trackpad01プロファイルをどう検証するかをまとめます。

### 5.1 処理段の構成

1レポートは次の順に処理されます。

1. **12 byte一括読出し** — GestureからFinger 1 Areaまでを1回のI2C通信で確定させます。分割して読むと、1イベントに通信窓が2回必要になり、片方を取り逃す可能性があります。
2. **接触の整合性判定** — `fingers > 0`、X/Yが`0xFFFF`でない、strengthとareaが非ゼロであることをまとめて確認します。
3. **初回接触の判定** — 無効な座標から接触を開始しません。接触成立後の一時的な無効フレームは、接触を終わらせず直前値を保持します。
4. **ラバーバンドdeadband** — 出力は入力の後方deadbandピクセルに追従します。
5. **3サンプル中央値** — 単発の外れ値を除去します。

絶対モードと相対モードは2〜5を共有します。共通のfiltered X/Yを作った後でのみ、絶対座標の報告と相対deltaへの変換へ分岐します。この境界を前へ動かすと、同じ指の動きに対して2つのモードが異なる座標経路を持つため、変更時は`test_absolute_relative_parity`で同等性を確認してください。

### 5.2 パラメータ化と状態の境界

座標フィルターの調整値は、ビルド時のDevice Treeプロパティとして公開します。

| 実効設定 | 既定値 | 上書きするDevice Treeプロパティ |
|---|---:|---|
| ラバーバンド幅 | 8 | `jitter-deadband` |

bindingの既定値、`struct iqs7211e_config`の型、`DT_INST_PROP_OR`のfallbackは一致させます。固定の22 mm Trackpad01向けdriver既定値は`jitter-deadband = 8`です。パネル、電極、表面材などのハードウェア構成を変更する場合だけ再測定し、必要な値をboard overlayで上書きします。

deadband履歴や中央値履歴などの可変状態は`struct iqs7211e_data`に置き、読み取り専用のdevice configと混ぜません。

`stationary-report-interval-ms`と`stationary-report-layers`は座標フィルターではなく、絶対モードへ分岐した後の静止中再送だけを制御します。`touch-verify-interval-ms`は再送workから分離したセンサー生存確認であり、絶対・相対の両モードで再送周期やレイヤーに依存せず動作します。`stationary-report-layers = <1>`はlayer 1だけを表し、layer 0と1なら`<0 1>`と書きます。値はビットマスクではありません。

### 5.3 各段が存在する理由

#### ホスト側でIIRを重ねない理由

IQS7211Eはチップ内でMAVとDynamic IIRを実行します（datasheet 7.8）。ホスト側でさらにIIRを重ねると平滑化が二重になり、移動経路が縮んで遅れが増えます。このドライバーはチップ内の平滑化を最終結果として扱い、ホスト側IIRを持ちません。

#### deadbandを最小限にする理由

ラバーバンドdeadbandは静止中や移動中の微細な揺れを吸収しますが、値を上げると意図した動きにも同じ軸方向の遅れを加えます。実測した静止ノイズを抑えられる最小値を使い、追従性との釣り合いを取ります。TP Movementフラグによる追加ゲートは、固定22 mmパネルの実測で有意な改善がなく、確認報数を増やすと遅延と追いつきジャンプが増えたため使用しません。

### 5.4 固定パネル値の検証方法

#### Touch SET / CLEAR

閾値は`Threshold = Reference × (1 + Multiplier / 128)`です（datasheet 5.5.1）。乗数が大きいほど鈍くなります。

AZD123 4.3.1とAZD128 5.5.5の基本手順は次のとおりです。

1. 小さい指で**4チャネルの中間**を軽く押し、4つのdeltaが同程度になる位置へ置きます。
2. SETを4チャネルの最小値より下に置きます。
3. CLEARをSETより下げてヒステリシスを作ります。

ホバーが軽い接触と同じ水準まで届く場合、この手順と誤検出解除が両立しないことがあります。誤検出を解除するにはCLEARをホバー水準より上に置く必要がありますが、SETはCLEARより高くなければなりません。この衝突が起きた場合は、どちらを優先したかと理由を残してください。

#### jitter-deadband

下限は静止ログで観測した残留変位を吸収できる値です。上限は最小の意図的な移動を潰さない値です。静止ログと小さな円運動のログを同じ設定で再生し、静止時の移動量、移動中の経路長、最大ステップ、追従遅延を比較して決めます。

#### ATI

AZD123 4.2.1とAZD128 5.5.4に従い、次を確認します。

- ATI Compensationが0〜1023の中央付近にある
- ATI Errorフラグ（INFO_FLAGS bit 3）が立たない
- referenceが`ATI target ± Reference drift limit`の内側にある
- 接触時のdeltaが用途に足りる

Coarse divider / multiplierは通常AZD123表4.1のindex 0から動かさず、Fine dividerで調整します。Fine dividerは16未満に下げません。

#### X/Y Trim

AZD128 6.4の基準は、両軸で座標0と最大解像度へ到達できることです。四隅と四辺を個別に確認します。trimは1軸の両端へ同じ量だけ効くため、片端だけの非対称な余りは解消できません。

### 5.5 変更時の確認項目

1. 中央へ軽く触れて静止し、contactが1回、途中releaseが0回、出力移動がほぼ0であること
2. 軽いタップを反復し、contact数とrelease数が一致すること
3. 低速直線、円、高速往復で、途中release、IRQ/work/reportの欠落、I2C errorが0であること
4. `tests/filter/run.sh`が通り、絶対・相対のparityテストが成功すること
5. 最終ファームウェアのFLASH/RAMを変更前と比較すること

### 5.6 参照箇所

- [IQS7211E Datasheet](/docs/iqs7211e_datasheet.pdf): 5.5.1、5.6、5.7、7.5、7.8、11.9、Appendix A
- [AZD123 IQS721xy Trackpad User Guide](/docs/azd123_iqs721xy_trackpad_userguide.pdf): 4.2、4.3、4.6、5.2、5.3、5.6
- [AZD128 Gamepad Trackpad Design Guide](/docs/azd128-gamepad-trackpad-design-guide_v1.0.pdf): 5.5、6.2、6.3、6.4
