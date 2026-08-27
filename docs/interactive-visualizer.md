# Interactive flight visualizer

`visualizer-web`はThree.jsを描画adapterとして使い、Rust FDMの出力を一人称または
追従三人称で表示する。Three.js側にaircraft physics、servo、sensor、FBWを複製しない。

## 起動

Node.js 22.12以上を使用する。公開情報の基準日2026-08-22以前のversionへ固定している。

```powershell
cd visualizer-web
npm.cmd install
npm.cmd run sample
npm.cmd run dev
```

`http://127.0.0.1:4173/`を開く。`npm run dev`はRustの`interactive-bridge`もbuildする。
production buildだけを確認する場合は`npm.cmd run build`を使う。静的な`dist`だけでは
local Rust processを起動できないため、interactive modeには`server.ts`が必要である。

## 表示

- `Cockpit`: pilot位置相当から水面、水平線、platform、翼端を見る。
- `Chase`: 水平を保った追従cameraで機体姿勢と軌道を見る。
- `V`: camera切替、`H`: HUD切替、`Space`: replayのpause/play。
- default replayは`reports/run.csv`を20 Hzへdownsampleしたtracked sampleである。
- 任意の`sim-cli` CSVまたはinteractive logをfile inputから再生できる。

座標はNED/FRDからThree.jsの右手`Y-up`へ固定matrixで変換する。

```text
NED (north, east, down) -> Three (x, y, z) = (east, -down, -north)
body forward/right/down -> local -Z/+X/-Y
R_three = M R_ned M^T
```

Euler角をThree.jsの各rotation propertyへ直接代入しない。unit testはzero attitude、
+90 deg yaw、positive pitchを検証する。

## Pilot input

elevator/rudderそれぞれで次を独立に選べる。

- keyboard buttons（default: `W/S`、`A/D`）
- gamepad axis（default: axis 1、axis 0）
- gamepad buttons（default: D-pad 12/13、14/15）

keyは画面のbinding buttonを選んで次のkeyを押す。gamepadはaxis/button番号、反転、
dead zone、response exponentを変更できる。elevatorをbutton、rudderをstickとする混在も
可能で、設定はbrowser local storageへ保存する。

button入力は瞬時の0/100舵角にしない。押下中に`buttonRisePerSecond`で要求を増やし、
解放時は`buttonReturnPerSecond`でneutralへ戻す。反対button同時押し、window focus喪失、
gamepad disconnectではneutralとする。local serverも250 ms command timeout時にpilot要求を
neutralへ戻す。ただしbrowser/OSの入力処理は実機cockpit hardwareのvalidationではない。

実機入力方式は、拘束姿勢、手袋、振動、水しぶき、誤押下、固着、断線を含む操作試験を行い、
stickと大形momentary buttonを比較して決める必要がある。buttonの配置や触覚識別は本viewerの
keyboard/gamepad比較だけでは決めない。

## Manual / shared / automatic

`Auto authority`を0～100%で連続変更する。Rust側で舵ごとに次を計算する。

```text
mixed = (1 - autonomy) * manual + autonomy * automatic
```

- 0%: full manual。正規化pilot要求をmodelの最大舵角へ写す。
- 1～99%: shared control。pilot/automatic surface commandのauthority blend。
- 100%: full automatic。pilot要求はtelemetryへ残すがcommandへ反映しない。

縦automaticはactual RP2040 firmwareと共有する`fbw-control-core`を使う。横automaticは
sensor roll/roll-rate/yaw-rateからrudderを作るtraining-onlyのwing-level/rate dampingで、
実機同定済みFBWではない。混合後にmodelの舵角範囲でsaturateし、その後に共通のservo
rate/lag/deadband/quantizationと6DoF plantを通す。

live logはpilot、automatic、mixed、実舵角、autonomyを別列でdownloadできる。異なる
authorityを比較する場合は、同じ入力script、初期条件、model、windを使う必要がある。

neutral入力を30秒与えたdeterministic smoke runは次の通りだった。

| Auto authority | 接水時刻 | 北向距離 | 最大再浮上 |
| ---: | ---: | ---: | ---: |
| 0% | 4.37 s | 44.51 m | 0 m |
| 50% | 19.97 s | 209.62 m | 0 m |
| 100% | 23.64 s | 236.35 m | 0 m |

これはmanual-neutralと自動制御の差をsoftware上で確認した結果であり、飛距離予測や
controller safetyの実機妥当性を示さない。pilot skill比較には入力trajectoryを保存して
反復し、実flight logでmodelを更新する必要がある。

## MCU boundary

interactive live modeはhost Rust controller/FDMを動かす。縦制御algorithmはactual UF2と
同じcoreだが、現在の実機firmwareにはpilot input peripheralとrudder PWMがないため、
この経路をproduction-binary SILとは呼ばない。

actual UF2を使う既存`virtual-platform`のCSVはviewerでreplayできる。次段階は実機入力
hardwareを決め、virtual ADC/PWM/I2C inputとrudder outputを追加して、production UF2の
pilot-in-the-loopを閉じることである。これは`embedded-rust-playground`から引き継いだ
「実firmwareのMMIOをvirtual peripheralが受ける」境界を維持して行う。

## Verification

```powershell
npm.cmd run check
npm.cmd test
npm.cmd run build
# npm.cmd run dev を別terminalで起動してから
npm.cmd run smoke:live
```

testは座標変換、CSV補間、dead zone、button同時押し/neutral復帰、設定sanitizeを検証する。
smoke testはHTTP sampleとWebSocket経由のmanual/shared/auto telemetryを確認する。
