# ZMK設定 - Sage60 静電容量式キーボード

このリポジトリには、Sage60という名前の静電容量式キーボード用のZMK（Zephyr Mechanical Keyboard）設定が含まれています。

## 概要

Sage60は分割型の静電容量式キーボードで、ADC（アナログデジタルコンバータ）を使用してキー入力を検出します。このリポジトリには以下の要素が含まれています：

- 静電容量式キースキャンドライバ（EC Matrixドライバ）
- 秋月電子製 NRF52840ブレイクアウトボード用のカスタムボード定義
- Sage60キーボード用のシールド定義
- キーマップ設定

## リポジトリ構造

```
config/
├── boards/
│   ├── arm/
│   │   └── ae_nrf52840/         # 秋月電子製NRF52840ボード定義
│   └── shields/
│       ├── sage60/              # 通常のSage60シールド
│       ├── sage60_ec/           # EC版Sage60シールド
│       ├── sage60_xiao/         # Seeed XIAO対応版
│       └── sage60_xiao_ec/      # Seeed XIAO対応のEC版
├── dts/                         # デバイスツリーの追加ファイル
└── sage60_drivers/              # カスタムドライバ
    └── kscan/
        ├── kscan_ec_matrix.c    # 静電容量式キースキャンドライバ
        └── Kconfig              # ドライバ設定
```

## 静電容量式キーボードの仕組み

Sage60 ECは静電容量の変化を検出してキー入力を判断します。このキーボードでは以下の技術を使用しています：

1. マルチプレクサを使用して複数の静電容量プレートを切り替え
2. ADCを使用して静電容量の変化を測定
3. カスタムドライバ（kscan_ec_matrix）で値を処理

## ADC設定

静電容量式キーボードでは、ADCが正しく設定されていることが重要です：

```dts
&adc {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;
    
    channel@0 {
        reg = <0>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 50)>;
        zephyr,resolution = <12>;
        zephyr,input-positive = <NRF_SAADC_AIN7>; /* P0.02 */
    };
};
```

## 秋月電子製 NRF52840ボード

このプロジェクトでは、秋月電子製のNRF52840ブレイクアウトボードをコントローラとして使用しています。このボードはArduino Pro Microのピン配置と互換性を持つように設定されています：

```dts
pro_micro: connector {
    compatible = "arduino-pro-micro";
    #gpio-cells = <2>;
    gpio-map-mask = <0xffffffff 0xffffffc0>;
    gpio-map-pass-thru = <0 0x3f>;
    gpio-map 
            = <0 0 &gpio0 4 0>,	/* D14 A0 */
            <1 0 &gpio0 5 0>,	/* D15 A1 */
            /* 省略 */
            <4 0 &gpio0 2 0>,	/* D18 A4 (ADC AIN7) */
            /* 省略 */
};
```

## キーマップ

Sage60はデフォルトで以下のレイヤーを持ちます：

1. デフォルトレイヤー
2. ロワーレイヤー（Lower）
3. レイズレイヤー（Raise）
4. トリプルレイヤー（Lower + Raiseを同時押し）

## ビルド方法

このリポジトリは[ZMK Firmware](https://zmk.dev/)のユーザー設定リポジトリとして設計されています。以下のコマンドでビルドできます：

```bash
west build -b ae_nrf52840 -d build/sage60_ec_left -- -DSHIELD=sage60_ec_left
west build -b ae_nrf52840 -d build/sage60_ec_right -- -DSHIELD=sage60_ec_right
```

## 開発者情報

このキーボードは静電容量式スイッチを使用し、カスタムドライバでADCを介して入力を検出します。 