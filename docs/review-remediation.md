# 全体レビュー修正記録 — 2026-09-08

## 結論と適用範囲

対象は実RP2040 firmwareを実rp2040js上で実行するFBW電装実験環境。
`embedded-rust-playground`と同じ実HAL/MMIO境界を維持し、ホスト制御へのfallbackは使わない。
飛行距離、失速安全性、実機のcycle timingのvalidationを完了した意味ではない。
旧completion-auditや飛行数値は履歴であり、この修正後の実験結果と混ぜない。

## 指摘への対応

| 問題 | 現行の契約・回帰試験 |
| --- | --- |
| SDP810再初期化が実仕様と不整合 | stop→500us待機→start、最初の8msはread NACK。開始前read・再start拒否を試験 |
| CCレジスタをサーボ出力と誤認 | GPIO16/17のエッジから周期・幅を検査。誤mux、無効出力、誤分周/TOPを実rp2040jsで拒否 |
| strictモデルがMCU経路で継続 | 共通終了条件＋PlantSession停止ラッチ。実plant-bridge protocol試験 |
| auto指令逆算・時刻混在 | 実firmware UART1記録をCRC検査。auto/mixed/safe/PWM受信値を区別 |
| 気圧の値変化をfresh判定に使用 | 取得sequence＋時刻。fresh同値、保持、欠測、不定周期、wrapを試験 |
| 古いsocketイベントが新状態を破壊 | TEA session IDとsocket identity、終了状態の遷移制限 |
| 累積命令上限で長時間停止 | 起動・step別watchdog。120s actual-UF2 runtime fixture |

subagentは差分に加えて全体の責務・実験証拠をレビューした。追加発見した以下も修正した。

- MCUが未来のplant測定を読む順序：両サブシステムは区間開始時の保持入力で進め、終了後に観測を公開。
- DPS310二重sample-and-hold：連続トランスデューサ入力をMCUデバイスが一度だけ取得。
- 欠測で実経過時間を過小評価：取得時刻差で推定し、制御filter/slewには実loop時間を渡す。
- ラダーが安全ゲートを迂回：両軸が共通のarming/hold/failsafe/recovery状態を持つ。manual authorityは明示的に別。
- ボタンのブラウザ整形をGPIO変換で喪失：ボタンはbinary入力、analogはanalog整形。誤解を招くrate設定を除去。
- 解析前間引きで振動・最大値を喪失：全sampleをUUID単位でIndexedDB保存。別tab/reloadで同じ記録を取得。
- 終了理由を着水に捏造：構造化terminal reasonを保持し、理由なしexitを正常着水と扱わない。
- 南向き軌跡の欠落、focus喪失後gamepad再入力、割当キーの既定scroll：表示範囲と入力境界を修正。

## 時間とログ

通常のWebはCPU倍率1のみ受理し、batchの既定も1。1より大きい倍率はCPUだけの時間関係を
変える負荷実験であり、PCが実時間に追いつくように見せる目的では使わない。
画面はwall-clock不足とvirtual deadlineを別に表示し、cycle timing UNVALIDATEDを常時明示。

UART1 GPIO8/9、1Mbaud、8N1の通常firmware記録はFBW2固定56byte。
little-endianのmagic4byte、sequence u32、loop開始timer us u32、valid flags u32、
9個のf32（pilot2軸、authority、auto2軸、safe elevator、mixed2軸、safe rudder）、CRC32/IEEE。
CRCは最後の4byteを除く全体を対象とする。invalid autoの0は欠測表現であり、有効性falseと組で扱う。

`release_mcu_time_us`でpreflightとplant時刻の原点を対応付ける。
record/PWMは`plant_interval_start_s`時点で利用可能だったもの、plant位置は区間終了時の値。
PWMは完了パルスのfalling-edge時刻を別に保存する。同じCSV行を同時取得だと解釈しない。
10ms保持結合による離散化遅延は残るので、制御帯域に対するstep収束確認が必要。
UF2/model/plantのSHA256をsummaryとWeb記録へ保持し、旧binaryの結果を現行の証拠と混ぜない。

## 検証手順

今回の実行結果：Rust 55 unit＋1 protocol integration＋1 doctest、UART encoder 2 tests、
virtual platform 16 tests、Web 35 tests、実ブラウザE2E 2 casesが成功。
E2Eは実UF2からのbutton/manual入力反映、再接続、Replay隔離、IndexedDB別tab/reloadを含む。
12,001個のfull evidence sampleを実IndexedDBに保存・復元し、4Kでの解析表示も確認した。
3種の実UF2復旧試験は同一artifact hashで再初期化0/1/4回、全て再arming・deadline超過0。
I2C-stallは実watchdog発火による明示的な実験停止を確認した。
120秒・CPU倍率1の実UF2容量試験は8,431,789,883命令、deadline超過0で終了した。
この容量試験のUF2 SHA256は`3d146c9fc812519126d59aee903ecb62824870efd47df6af7e85c81b4dee2c1f`。
通常飛行条件での120秒滞空や、実機cycle精度を証明する値ではない。

ブラウザskillの接続先がなかったため、Playwrightの独立headless ChromiumでE2Eを実施した。
実VR機材や人間による操作性評価の代わりではない。Viteの567kB程度のchunk警告は残る。

```powershell
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
npm.cmd run build:platform --prefix visualizer-web
npm.cmd run check --prefix virtual-platform
npm.cmd test --prefix virtual-platform
npm.cmd run check --prefix visualizer-web
npm.cmd test --prefix visualizer-web
npm.cmd run build --prefix visualizer-web
npm.cmd run test:e2e --prefix visualizer-web
```

`virtual-platform`で`node --import tsx src/long-run-check.ts`を実行すると、着水が120秒の
実行確認を妨げない高高度fixtureで実UF2を動かす。このfixtureは記録・実行容量試験専用で、
鳥人間の発進条件や飛行性能を主張するものではない。
同じdirectoryの`node --import tsx src/recovery-check.ts`と
`node --import tsx src/watchdog-stall-check.ts`で復旧・固着試験を新規実行する。
UART encoderは`rustc --test firmware/fbw-rp2040/src/telemetry.rs --edition=2024 -o target/telemetry-tests.exe`
と`target/telemetry-tests.exe`でhost上でも検査する。

## 実機なしでは閉じない境界

実センサの全モード、電圧・電流・EMI・サーボ負荷、故障時実舵角は未同定。
GPIOパルス喪失は実験エラーとして表示・停止するが、物理サーボが安全位置へ動くことは保証しない。
watchdog/reset境界は[electrical contract](electrical-contract-review.md)を参照。
対象機の高迎角データ、慣性・安定微係数・構造応答、RC/実機logのholdout validation、VR実機QAは未完了。
