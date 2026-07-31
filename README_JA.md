# zmk-pmw3610-driver — Dev-v0.3_inertial-scroll

[English](README.md) | 日本語

## Credits & Respect

This module is based on [badjeff/zmk-pmw3610-driver](https://github.com/badjeff/zmk-pmw3610-driver).

badjeff built upon [ufan's zmk pixart sensor drivers](https://github.com/ufan/zmk/tree/support-trackpad) and [inorichi's zmk-pmw3610-driver](https://github.com/inorichi/zmk-pmw3610-driver) to create a well-structured PMW3610 driver for ZMK v0.3 — with split peripheral support, per-sensor DTS configuration, and shared SPI bus compatibility. His work laid the foundation for trackball integration in ZMK, and this branch would not exist without it. Deep respect and gratitude to badjeff for his contributions to the community.

このブランチはその実装をベースに、以下の追加改良を加えたものです。

---

## このブランチ (Dev-v0.3_inertial-scroll) の概要

### 安定性・電源管理の改善

- **レベルトリガ割り込み**: エッジトリガではなくレベルトリガ (`GPIO_INT_LEVEL_ACTIVE`) を採用し、割り込み無効期間中のモーションイベント取りこぼしを防止。
- **フェイルセーフ初期化**: SPI 初期化失敗時に最大3回リトライし、連続失敗時はデフォルト1秒のバックオフ後に初期化を最初からやり直す。バックオフ時間は `CONFIG_PMW3610_INIT_RETRY_BACKOFF_MS` で変更できる。
- **FAULT リカバリ & ジャンプ防止**: FAULT 検知時に蓄積済みの移動量 (`dx`, `dy`) と慣性状態を破棄し、復帰後のカーソル暴走を防止。
- **入力キュー保護**: X/Y の斜め移動を1つの同期レポートに保ち、X投入後は対応するYが入るまで待つことで、未完了イベントが後から混入することを防止。
- **慣性状態の排他制御**: 慣性ワーク、トグル、FAULT復旧が同時に走っても、停止後に古い慣性ワークが状態を再生成しないよう保護。
- **IDLE 省電力の修正**: `force_awake_4ms_mode` が IDLE 移行後も4ms レートを維持し続けるバグを修正。IDLE 時は正しく 8ms デフォルトレートに落ちて省電力動作する。

### ドライバーサイド慣性スクロール

PMW3610 をスクロールデバイスとして使うレイヤーで、指を離した後もスクロールが慣性で継続する機能をドライバ側で実装しています。

---

## インストール

### 1. west.yml への追加

`config/west.yml` に以下を追加します：

```yaml
manifest:
  remotes:
    - name: razilyis
      url-base: https://github.com/razilyis
  projects:
    - name: zmk-pmw3610-driver
      remote: razilyis
      revision: Dev-v0.3_inertial-scroll
  self:
    path: config
```

### 2. board overlay への追加

`<board>.overlay` にセンサーの設定を追記します（ピン番号は基板に合わせて変更してください）：

```dts
&pinctrl {
    spi0_default: spi0_default {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                    <NRF_PSEL(SPIM_MOSI, 0, 17)>,
                    <NRF_PSEL(SPIM_MISO, 0, 17)>;
        };
    };
    spi0_sleep: spi0_sleep {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                    <NRF_PSEL(SPIM_MOSI, 0, 17)>,
                    <NRF_PSEL(SPIM_MISO, 0, 17)>;
            low-power-enable;
        };
    };
};

#include <zephyr/dt-bindings/input/input-event-codes.h>

&spi0 {
    status = "okay";
    compatible = "nordic,nrf-spim";
    pinctrl-0 = <&spi0_default>;
    pinctrl-1 = <&spi0_sleep>;
    pinctrl-names = "default", "sleep";
    cs-gpios = <&gpio0 20 GPIO_ACTIVE_LOW>;

    trackball: trackball@0 {
        status = "okay";
        compatible = "pixart,pmw3610";
        reg = <0>;
        spi-max-frequency = <2000000>;
        irq-gpios = <&gpio0 6 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        cpi = <600>;
        evt-type = <INPUT_EV_REL>;
        x-input-code = <INPUT_REL_X>;
        y-input-code = <INPUT_REL_Y>;

        /* ドリフトフィルタ: 0 で無効化 */
        motion-threshold = <1>;

        /* 異常な単発移動量を破棄（任意、デフォルト 512） */
        max-motion-delta = <512>;

        /* 蓄積後の1レポートを制限（任意、デフォルト 2047） */
        max-report-delta = <2047>;

        /* 慣性スクロール（任意） */
        inertial-scroll;
        inertial-scroll-layers = <6 7>;   /* 有効にするレイヤー番号。省略時は全レイヤーで有効 */
        inertial-scroll-gain-pct = <130>;
        inertial-scroll-decay-pct = <99>;
        inertial-scroll-interval-ms = <10>;
        inertial-scroll-threshold = <4>;

        /* 省電力制御（任意） */
        force-awake;          /* ACTIVE 時はセンサーを常時起動 */
        force-awake-4ms-mode; /* ACTIVE 時に 4ms サンプリングを強制（USB 接続で 250Hz が必要な場合） */

        // swap-xy;   /* 任意: XY 軸の入れ替え */
        // invert-x;  /* 任意: X 軸の反転 */
        // invert-y;  /* 任意: Y 軸の反転 */
    };
};

/ {
    trackball_listener {
        compatible = "zmk,input-listener";
        device = <&trackball>;
    };
};
```

### 3. shield config への追加

`<shield>.conf` に以下を追記します：

```conf
CONFIG_SPI=y
CONFIG_INPUT=y
CONFIG_ZMK_POINTING=y
CONFIG_PMW3610=y
# CONFIG_PMW3610_REPORT_INTERVAL_MIN=12  # 任意: 最小レポート間隔 (ms)
# CONFIG_PMW3610_LOG_LEVEL_DBG=y         # 任意: デバッグログ
# CONFIG_PMW3610_INIT_POWER_UP_EXTRA_DELAY_MS=300  # トラブルシューティング参照
```

---

## DTS プロパティ一覧

### 基本設定

| プロパティ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `irq-gpios` | phandle-array | (必須) | モーション割り込み GPIO |
| `cpi` | int | 600 | カウント/インチ（200〜3200、ステップ 200） |
| `evt-type` | int | (必須) | 入力イベント種別（`INPUT_EV_REL` など） |
| `x-input-code` | int | (必須) | X 軸の入力コード |
| `y-input-code` | int | (必須) | Y 軸の入力コード |
| `motion-threshold` | int | 1 | ドリフトフィルタ閾値。XとYの絶対値が両方ともこの値以下のサンプルを破棄。`0` で無効 |
| `max-motion-delta` | int | 512 | XまたはYの絶対値がこの値以上の単発サンプルを破棄し、異常なカーソルジャンプや慣性生成を防ぐ（1〜2048） |
| `max-report-delta` | int | 2047 | 蓄積後に1レポートで送る絶対値を制限し、超過分を後続レポートとして放出せず破棄する（1〜2047） |
| `swap-xy` | boolean | — | XY 軸を入れ替える |
| `invert-x` | boolean | — | X 軸を反転する |
| `invert-y` | boolean | — | Y 軸を反転する |

### 省電力制御

| プロパティ | 型 | 説明 |
|---|---|---|
| `force-awake` | boolean | ZMK が ACTIVE 状態の間センサーを常時起動。IDLE/SLEEP 移行後は通常のダウンシフトに戻る |
| `force-awake-4ms-mode` | boolean | `force-awake` 有効時に 4ms サンプリング（250Hz）を強制。USB 直結で高レートが必要な場合に使用 |

### 慣性スクロール

| プロパティ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `inertial-scroll` | boolean | — | 慣性スクロールを有効化する |
| `inertial-scroll-gain-pct` | int | 130 | 平滑化したジェスチャー速度から慣性初速を生成する際のゲイン（%）。大きいほど速くなる |
| `inertial-scroll-decay-pct` | int | 99 | 毎 tick の速度減衰率（%）。小さいほど早く止まる |
| `inertial-scroll-decay-basis-points` | int | 0 | 任意の高精度減衰率（0.01%単位）。`9920`は99.20%。0なら`decay-pct`を使用 |
| `inertial-scroll-interval-ms` | int | 10 | 慣性スクロールの合成レポート間隔（ms） |
| `inertial-scroll-threshold` | int | 4 | 慣性スクロールを停止する速度閾値（Q8 固定小数点単位） |
| `inertial-scroll-max-velocity` | int | 32 | 慣性初速の上限（1 tickあたりのステップ数） |
| `inertial-scroll-max-duration-ms` | int | 1800 | 1回の慣性スクロールを継続できる最大時間（ms） |
| `inertial-scroll-fade-duration-ms` | int | 250 | 最大継続時間の直前に速度を線形フェードする時間（ms）。`0`で無効 |
| `inertial-scroll-layers` | array | — | 慣性スクロールを有効にするレイヤー番号のリスト。省略時は全レイヤーで有効 |
| `scroll-direction-toggle` | boolean | — | `inertial-scroll`を使わないスクロール専用センサーも方向トグルの対象にする |
| `vertical-scroll-uses-x-axis` | boolean | false | 90度回転して搭載したセンサーで、生のX軸を縦スクロール方向トグルの対象にする |

`inertial-scroll-layers` は、同じ PMW3610 をポインタとスクロールで兼用する場合に使います。スクロールレイヤーの番号だけを指定してください：

```dts
inertial-scroll-layers = <6 7>;
```

慣性初速は最後の1レポートだけではなく、実際のレポート間隔で時間正規化した直近のジェスチャー速度から算出します。新しいジェスチャーの最初のレポートは、ACTIVE中ならRUN周期とレポート間引き設定、IDLE後なら経過時間から推定したRUN/REST周期を使用します。これにより短いフリックの勢いを保ちながら、REST復帰時に蓄積デルタから過剰な慣性が生成されるのを防ぎます。加速には素早く追従し、減速にはゆっくり追従するため、速いフリックの終端で指が自然に減速しても勢いが残ります。80msを超えて入力が途切れた場合や方向が反転した場合は、新しいジェスチャーとして速度履歴をリセットします。

センサーを90度回転して搭載し、生のX軸を縦スクロールへ変換する場合は、センサーノードへ `vertical-scroll-uses-x-axis;` を追加してください。

### 低速カーソル安定化

| プロパティ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `low-speed-stabilizer` | boolean | — | 低速マイクロモーション安定化を有効化する |
| `low-speed-stabilizer-threshold` | int | 1 | マイクロモーションとして扱う最大絶対値 |
| `low-speed-stabilizer-timeout-ms` | int | 30 | 無入力後に方向履歴をリセットする時間。停止後の最初の微小入力は保留せず出力する |

同方向の小さな入力は確認後に距離を保持したまま出力し、単発の逆方向入力は一旦保留します。次の入力が元の方向へ戻ればノイズとして相殺し、逆方向が続けば意図した方向転換としてまとめて出力します。`inertial-scroll-layers`で指定したスクロールレイヤーでは自動的にバイパスされます。

### 安全・復旧関連のKconfig

| 設定 | デフォルト | 説明 |
|---|---|---|
| `CONFIG_PMW3610_INPUT_REPORT_TIMEOUT_MS` | 0 | 入力キューへ1イベントを追加するときの最大待機時間（0〜20ms）。0はノンブロッキング送信 |
| `CONFIG_PMW3610_INPUT_RETRY_TIMEOUT_MS` | 50 | 未送信フレームを保持して再試行する最大時間。超過後は移動量を破棄する |
| `CONFIG_PMW3610_INIT_RETRY_BACKOFF_MS` | 1000 | 初期化を3回再試行しても失敗した場合、初期化全体をやり直すまでの待機時間（100〜10000ms） |
| `CONFIG_PMW3610_RECOVERY_DELAY_MS` | 20 | SPI・FAULT・IRQ異常後にランタイム復旧を始めるまでの短い待機時間 |
| `CONFIG_PMW3610_STUCK_IRQ_TIME_MS` | 20 | MOTなしでIRQ activeが継続した場合に固着と判定するまでの最短時間 |
| `CONFIG_PMW3610_WORKQUEUE_STACK_SIZE` | 1536 | PMW3610専用workqueue用に確保するスタックサイズ |
| `CONFIG_PMW3610_WORKQUEUE_PRIORITY` | 10 | PMW3610専用workqueueのプリエンプティブ優先度 |

---

## 慣性スクロールのトグルキー

キーマップから慣性スクロールをオン/オフするゼロパラメータのビヘイビアを利用できます。

`.keymap` ファイルに以下を追加します：

```dts
#include <behaviors/pmw3610_inertia_toggle.dtsi>
```

任意のレイヤーのキーに割り当てます：

```dts
&pmw3610_inertia_toggle
```

## 縦スクロール方向のトグルキー

縦スクロールの正転・逆転を切り替えるゼロパラメータのビヘイビアを利用できます。

`.keymap` ファイルに以下を追加します：

```dts
#include <behaviors/pmw3610_scroll_direction_toggle.dtsi>
```

任意のキーに割り当てます：

```dts
&pmw3610_scroll_direction_toggle
```

Split構成ではGlobal BehaviorとしてCentralとPeripheralの両方へ配送されます。
単純な反転命令ではなく、Centralで確定したON/OFF状態を明示的に配送します。
`inertial-scroll-layers` が指定されたセンサーでは対象レイヤーだけを反転するため、
通常のカーソル方向には影響しません。方向トグルは `inertial-scroll` が有効なセンサーだけを対象とし、通常のポインター専用センサーには影響しません。慣性を使わないスクロール専用センサーを対象にする場合だけ、センサーノードへ `scroll-direction-toggle;` を追加してください。切替時には進行中の慣性を停止します。

## 横スクロール方向のトグルキー

横スクロールの正転・逆転を切り替えるゼロパラメータのビヘイビアを利用できます。

```dts
#include <behaviors/pmw3610_horizontal_scroll_direction_toggle.dtsi>
```

任意のキーに割り当てます：

```dts
&pmw3610_horizontal_scroll_direction_toggle
```

縦方向と同様にGlobal Behaviorとして配送され、対象センサーの通常スクロールと慣性スクロールの両方へ反映されます。対象条件は縦方向と同じです。`vertical-scroll-uses-x-axis` が指定されたセンサーでは、X軸を縦、Y軸を横として扱います。未指定時はY軸が縦、X軸が横です。

## 制御Behaviorの初期状態

設定が保存されていない初回起動時の状態は以下です。

| 制御 | 初期状態 |
|---|---|
| 慣性スクロール | ON |
| 縦スクロール方向の反転 | OFF |
| 横スクロール方向の反転 | OFF |

`CONFIG_SETTINGS=y` で保存済みの状態がある場合は、その値を起動時に復元します。

## 片側センサー構成

PMW3610デバイスの列挙は0台、1台、複数台のすべてに対応します。Behavior制御部は `CONFIG_PMW3610` とは独立してビルドされるため、センサーがCentralだけ、Peripheralだけ、または両側にある構成を利用できます。センサーのない側は状態同期だけを担当し、搭載側のセンサーへ設定を適用します。

Split PeripheralはCentralのキーマップレイヤーを直接参照できないため、アクティブレイヤーを制御Behavior経由で同期します。これによりPeripheral側だけにセンサーがある場合も `inertial-scroll-layers` が機能します。起動時とレイヤー・トグル変更時に同期し、失敗時は有限回再試行します。常時5秒ポーリングは行わないため、再試行終了後に再接続した場合は、次のレイヤー変更またはトグル操作で再同期されます。

このレイヤー同期は `pmw3610_inertia_toggle` Behaviorを同期経路として使用します。Split Peripheral上のセンサーで `inertial-scroll-layers` を使う場合は、`pmw3610_inertia_toggle.dtsi` をincludeし、キーマップから `&pmw3610_inertia_toggle` を参照してBehavior nodeが有効になるようにしてください。各dtsiのBehavior nodeには `/omit-if-no-ref/` が指定されているため、includeするだけで参照がない場合はビルド時に除去されます。

### keymap-editor をお使いの場合

[nickcoutsos/keymap-editor](https://github.com/nickcoutsos/keymap-editor) は外部 west モジュールのビヘイビアをUI経由で追加できません。ただし、既に `.keymap` に記載されているバインディングはそのまま保持されます。

**ワークアラウンド**: configリポジトリ側の `.keymap` にBehavior nodeを定義するか、
`&pmw3610_inertia_toggle` / `&pmw3610_scroll_direction_toggle` /
`&pmw3610_horizontal_scroll_direction_toggle` を手書きで割り当ててください。
外部westモジュールを直接解析しないEditorでも既存バインディングは保持されます。
