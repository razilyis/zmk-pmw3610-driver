# zmk-pmw3610-driver — Dev-v0.4_inertial-scroll

[English](README.md) | 日本語

## Credits & Respect

This module is based on [badjeff/zmk-pmw3610-driver](https://github.com/badjeff/zmk-pmw3610-driver).

badjeff built upon [ufan's zmk pixart sensor drivers](https://github.com/ufan/zmk/tree/support-trackpad), [inorichi's zmk-pmw3610-driver](https://github.com/inorichi/zmk-pmw3610-driver), and the Zephyr PMW3610 driver to create a well-structured PMW3610 driver for ZMK — with split peripheral support, per-sensor DTS configuration, and shared SPI bus compatibility. His work laid the foundation for trackball integration in ZMK. Deep respect and gratitude to badjeff and the contributors.

このブランチは badjeff の高精度・低遅延なカーソル追従コードをベースに、**ZMK v0.4 (Zephyr 4.1)** 対応と**ドライバーサイド慣性スクロール・制御Behavior** を統合したものです。

---

## このブランチ (Dev-v0.4_inertial-scroll) の概要

### 🟢 ZMK v0.4 (Zephyr 4.1) への完全対応
- **DTS Compatible**: `pixart,pmw3610-alt`（Zephyr 4.1 上流ドライバとの衝突を回避。従来の `pixart,pmw3610` も互換サポート）
- **Kconfig プレフィックス**: `CONFIG_PMW3610_ALT_*`（`CONFIG_PMW3610_*` も自動フォールバック）
- **Zephyr 4.1 Input Subsystem**: 新しい入力基盤および Device Driver API に適合

### 🟢 最高精度の 1:1 カーソル追従性（通常ポインティング時）
- 通常のカーソル操作時は、余分なフィルタや遅延処理を挟まず、badjeff 本家と全く同一のダイレクトな高速サンプリング（ゼロ遅延・完全な滑らかさ）で動作します。

### 🟢 ドライバーサイド慣性スクロール（スクロールレイヤー時）
- **心地よい滑り心地**: スクロールレイヤーでトラックボールをフリックした際、指を離した後も指数関数的な減速を伴ってなめらかにスクロールが継続します。
- **ジェスチャー速度の正規化**: 最後の1サンプルだけでなく、レポート間隔で時間正規化した直近のフリック速度から慣性初速を算出。REST 復帰時の過剰な飛び出し（暴走）を防ぎつつ、素早いフリックの勢いを保持します。
- **継続時間・フェード制御**: 最大持続時間（デフォルト: 1800ms）と終了前の線形フェードアウト（250ms）により、不自然な急停止のない自然な減速を実現。

### 🟢 リアルタイム制御ビヘイビア & Split同期
- キーマップからいつでも慣性スクロールの ON/OFF、縦スクロール・横スクロールの正転/反転をトグル切り替え可能。
- Split 構成（Central ↔ Peripheral 間）でのレイヤー状態および制御状態の双方向同期に対応。

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
      revision: Dev-v0.4_inertial-scroll
  self:
    path: config
```

### 2. board overlay への追加

`<board>.overlay` にセンサーの設定を追記します（ピン番号は基板に合わせて変更してください）：

```dts
#include <zephyr/dt-bindings/input/input-event-codes.h>

&spi0 {
    status = "okay";
    compatible = "nordic,nrf-spim";
    pinctrl-0 = <&spi0_default>;
    pinctrl-1 = <&spi0_sleep>;
    pinctrl-names = "default", "sleep";
    cs-gpios = <&gpio0 9 GPIO_ACTIVE_LOW>;

    trackball: trackball@0 {
        status = "okay";
        compatible = "pixart,pmw3610-alt";
        reg = <0>;
        spi-max-frequency = <2000000>;
        irq-gpios = <&gpio0 2 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        cpi = <400>;
        evt-type = <INPUT_EV_REL>;
        x-input-code = <INPUT_REL_X>;
        y-input-code = <INPUT_REL_Y>;

        /* 慣性スクロール */
        inertial-scroll;
        inertial-scroll-layers = <6 7>;   /* 有効にするレイヤー番号。省略時は全レイヤーで有効 */
        inertial-scroll-gain-pct = <130>;
        inertial-scroll-decay-pct = <99>;

        /* 省電力制御 */
        force-awake;          /* ACTIVE 時はセンサーを常時起動 */

        // swap-xy;   /* 任意: XY 軸の入れ替え */
        // invert-x;  /* 任意: X 軸の反転 */
        // invert-y;  /* 任意: Y 軸の反転 */
    };
};
```

### 3. shield config への追加

`<shield>.conf` に以下を追記します：

```conf
CONFIG_SPI=y
CONFIG_INPUT=y
CONFIG_ZMK_POINTING=y
CONFIG_PMW3610_ALT=y
CONFIG_PMW3610_ALT_SMART_ALGORITHM=y
# CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN=15  # 任意: 最小レポート間隔 (ms)
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
| `max-motion-delta` | int | 512 | XまたはYの絶対値がこの値以上の単発サンプルを破棄し、異常なカーソルジャンプを防ぐ（1〜2048） |
| `max-report-delta` | int | 2047 | 蓄積後に1レポートで送る絶対値を制限（1〜2047） |
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

---

## 制御Behavior（トグルキー）

キーマップから各機能をオン/オフできるゼロパラメータのビヘイビアを利用できます。

| ビヘイビア名 | 説明 |
|---|---|
| `&pmw3610_inertia_toggle` | 慣性スクロールの有効 / 無効をトグル切り替え |
| `&pmw3610_scroll_direction_toggle` | 縦スクロール方向（正転 / 反転）をトグル切り替え |
| `&pmw3610_horizontal_scroll_direction_toggle` | 横スクロール方向（正転 / 反転）をトグル切り替え |

### [Keymap Editor (nickcoutsos)](https://github.com/nickcoutsos/keymap-editor) での使用

Web 版 Keymap Editor でビヘイビアを UI 選択できるようにするため、`.keymap` の `behaviors { ... }` ブロック内に定義します：

```dts
/ {
    behaviors {
        pmw3610_inertia_toggle: pmw3610_inertia_toggle {
            compatible = "zmk,behavior-pmw3610-inertia-toggle";
            #binding-cells = <0>;
            label = "PMW3610_INERTIA_TOGGLE";
            display-name = "PMW3610 Inertia Toggle";
        };

        pmw3610_scroll_direction_toggle: pmw3610_scroll_direction_toggle {
            compatible = "zmk,behavior-pmw3610-scroll-direction-toggle";
            #binding-cells = <0>;
            label = "PMW3610_SCROLL_DIRECTION_TOGGLE";
            display-name = "PMW3610 Scroll Direction Toggle";
        };

        pmw3610_horizontal_scroll_direction_toggle: pmw3610_horizontal_scroll_direction_toggle {
            compatible = "zmk,behavior-pmw3610-horizontal-scroll-direction-toggle";
            #binding-cells = <0>;
            label = "PMW3610_HORIZONTAL_SCROLL_DIRECTION_TOGGLE";
            display-name = "PMW3610 Horizontal Scroll Direction Toggle";
        };
    };
};
```

---

## 制御Behaviorの初期状態

初回起動時のデフォルト状態は以下です：

| 制御 | 初期状態 |
|---|---|
| 慣性スクロール | ON |
| 縦スクロール方向の反転 | OFF |
| 横スクロール方向の反転 | OFF |

`CONFIG_SETTINGS=y` が有効な場合は、トグルで変更した状態が自動的にフラッシュメモリへ保存され、再起動後も復元されます。
