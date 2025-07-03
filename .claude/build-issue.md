* 「west build -s /workspaces/zmk/app -d "./build/xiao_ec_left" -b "seeeduino_xiao_ble" -S "zmk-usb-logging" -- -DZMK_CONFIG="/workspaces/zmk-config/config" -DSHIELD="sage60_ec_left" -DZMK_EXTRA_MODULES="/workspaces/zmk-config"」のコマンドでビルドが失敗するため修正して
* このコマンドは.github/workflows/build.ymlによって生成されるコマンドです。
* zmk-configの形式を厳守すること
* zmkはdocker上で実行しているため、dockerコマンドを使用してコンテナ上で実行する
* コンテナは新規起動せず、既存のコンテナを使用する
* west updateは不要（/workspaces/zmk/appですでにwestは定義されている）
* コマンドの実行は/workspaces/zmk-config/配下で実行する
* zmkのdockerコンテナ構造：
* /workspaces/zmkに本体がある
* /workspaces/zmk-configにconfigフォルダがある
* zmk-configは既存のzmkを利用してビルドするため、west等の初期化は不要
* コマンドをコンテナ上の/workspaces/zmk-config配下で実行し、結果に対して修正が必要な箇所を修正する