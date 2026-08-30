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

先に`../virtual-platform`でも`npm.cmd install`を行う。`http://127.0.0.1:4173/`を開く。
`npm run dev`はRust `plant-bridge`、production RP2040 ELF/UF2もbuildする。
production buildだけを確認する場合は`npm.cmd run build`を使う。静的な`dist`だけでは
local Rust processを起動できないため、interactive modeには`server.ts`が必要である。

UIは1920×1080を1、3840×2160を2とするshortest-side比率でscaleする。FHDから4Kまで
同じ視野内占有率を保ち、ultrawideでは高さ側に合わせる。詳細は[UI scaling](ui-scaling.md)を
参照する。

## 表示

- `Cockpit`: pilot位置相当から水面、水平線、platform、翼端を見る。
- `Chase`: 水平を保った追従cameraで機体姿勢と軌道を見る。
- 水面には25 m間隔のstart中心AR distance ringを置き、50 mごとに距離を表示する。
- 水面は複数波長の時間変化する波として描画する。距離感のためのvisual-only表現であり、
  FDMの風、波、着水面には入力しない。
- directional-lightのshadow cameraは機体に追従させ、launch areaから離れた後もshadow mapの
  有効範囲から機体が外れないようにする。
- `V`: camera切替、`H`: HUD切替、`Space`: replayのpause/play。
- 起動時は`Interactive`をdefaultとし、production UF2とのlive loopを直ちに開始する。
- sample replayは`reports/run.csv`を20 Hzへdownsampleしたtracked sampleで、`Replay`を選ぶと利用できる。
- 任意の`sim-cli` CSVまたはinteractive logをfile inputから再生できる。

HUDではairspeedとwater面基準altitudeを大形表示し、altitudeが2 m未満になると色を変える。
経路角、迎角、roll、`T+ mm:ss.ss`を補助表示する。control-surface panelはelevator/rudderの
mixed commandを菱形、servo dynamics通過後のactual positionを丸で示し、数値も併記する。

flight phaseは`READY`、`LAUNCH`、`FLYING`、`WATER CONTACT`を区別する。replay dataが
水面へ到達する前に終わった場合は`END OF RECORDING`と表示し、着水と誤認させない。
着水時はflight time、startからの水平range、airspeedを中央に表示する。

座標はNED/FRDからThree.jsの右手`Y-up`へ固定matrixで変換する。

```text
NED (north, east, down) -> Three (x, y, z) = (east, -down, -north)
body forward/right/down -> local -Z/+X/-Y
R_three = M R_ned M^T
```

Euler角をThree.jsの各rotation propertyへ直接代入しない。unit testはzero attitude、
+90 deg yaw、positive pitchを検証する。

live plant sampleは100 Hzだが、WebSocket到着と60 Hz前後の描画frameは同期しない。Three.jsへは
到着時刻基準の30 ms jitter bufferを置き、前後2 sampleを位置・姿勢・舵角とも補間して渡す。
補間値は描画専用で、production UF2、sensor、servo、plant、保存flight logへfeedbackしない。
最新sampleがbufferへ届かなければ最後のsampleを保持し、遅いemulationを外挿や時間伸縮で隠さない。

## Optional WebXR cockpit

対応browser/headsetで`ENTER VR`を選ぶと、Three.jsの公式`VRButton`と`WebXRManager`を使って
stereoscopicなhead-tracked cockpit viewへ入る。flight、sensor、FBW、servoの経路は通常の
Interactive表示と同じであり、VR用physicsやcontrollerへ切り替えない。`setAnimationLoop`を使用し、
機体cockpit位置・姿勢をXR cameraの親rigへ与える。camera自体はWebXRに任せるため、頭の平行移動と
回転を機体姿勢で上書きしない。座位体験としてreference spaceは`local`を使用する。

XR session中はCockpitへ固定し、終了後は開始前のcamera modeへ戻す。WebXR availabilityは飛行の
TEA stateとは独立した判別可能unionで`checking / unavailable / ready / presenting`を表し、
非対応browser、HTTPS不足、capability check拒否を画面で区別する。

現在の範囲は3D cockpitとhead trackingまでである。HTML HUDはheadset内へ合成せず、XR controllerも
pilot入力へ割り当てない。操舵は引き続きkeyboard/gamepadからactual RP2040 UF2へ送る。tethered PC
headsetから同一PCの`http://127.0.0.1:4173/`を開く構成を想定する。standalone headsetなど別端末から
接続する場合、WebXRにはsecure contextが必要なので、HTTPS reverse proxy等を別途用意する必要がある。
headset固有のFOV、酔い、cockpit eye point、frame rateは実deviceで未検証であり、training適合性を
保証しない。

実装根拠:

- [Three.js VRButton](https://threejs.org/docs/pages/VRButton.html)
- [Three.js: How to create VR content](https://threejs.org/manual/en/how-to-create-vr-content.html)
- [Three.js WebXRManager](https://threejs.org/docs/pages/WebXRManager.html)
- [Three.js: WebXR basics](https://threejs.org/manual/en/webxr-basics.html)

## Pilot input

elevator/rudderそれぞれで次を独立に選べる。

- keyboard buttons（default: `W/S`、`A/D`）
- gamepad axis（default: axis 1、axis 0）
- gamepad buttons（default: D-pad 12/13、14/15）

keyは画面のbinding buttonを選んで次のkeyを押す。gamepadはaxis/button番号、反転、
dead zone、response exponentを変更できる。elevatorをbutton、rudderをstickとする混在も
可能で、設定はbrowser local storageへ保存する。

browser側のbutton表示値は`buttonRisePerSecond`で増減するが、actual-UF2 testではGPIO buttonを
active-lowの離散入力として渡すため、押下判定後はfull demandになる。反対button同時押し、window focus喪失、
gamepad disconnectではneutralとする。local serverも250 ms command timeout時にpilot要求を
neutralへ戻す。ただしbrowser/OSの入力処理は実機cockpit hardwareのvalidationではない。

実機入力方式は、拘束姿勢、手袋、振動、水しぶき、誤押下、固着、断線を含む操作試験を行い、
stickと大形momentary buttonを比較して決める必要がある。buttonの配置や触覚識別は本viewerの
keyboard/gamepad比較だけでは決めない。

## Manual / shared / automatic

`Auto authority`を0～100%で連続変更する。Web値はGPIO28/ADC2の12-bit countへ変換され、
actual RP2040 firmwareが舵ごとに次を計算する。

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

## MCU boundary and performance warning

interactive live modeはproduction UF2をrp2040jsへloadする。browser入力はvirtual ADC/GPIO、
BNO055/AS5600/SDP810/DPS310はvirtual I2C、出力はPWM0A/Bを必ず通る。host controllerへの
fallbackはない。Rust hostに残るのはservo/aircraft plantであり、制御演算ではない。

画面上部はbackendを`INTERACTIVE / RP2040JS`、`actual RP2040 UF2`と明示し、実時間倍率、
平均処理ms/update、累積lag、deadline missを表示する。平均処理が10 ms/updateを超える、
累積lagが100 msを超える、実時間倍率が0.9未満になる、またはfirmware deadline missを観測した場合は
`MCU EMULATION TOO SLOW — NOT REAL-TIME`を常時表示する。host controllerへ切り替えない。
実時間倍率だけは起動直後の1 sampleに過敏にならないよう、最初の0.5秒をsettling windowとする。
ただしrp2040jsのcycle timingはvalidation済みではなく、画面上のreal-timeはwall-clockへ
追従できるという意味だけである。

実行状態はThe Elm Architectureの`Model/Msg/update/view/Effect`へ分け、判別可能unionで
Replay、MCU接続中、実時間、性能不足、終了、失敗を表す。初期Modelも`mcu-connecting`であり、
HTMLの初期表示と一致する。sample CSVはbackgroundで読み込むが、成功してもReplayへ勝手に遷移しない。

## Verification

```powershell
npm.cmd run check
npm.cmd test
npm.cmd run build
# npm.cmd run dev を別terminalで起動してから
npm.cmd run smoke:live
```

testは座標変換、CSV補間、flight phase、flight clock、dead zone、
button同時押し/neutral復帰、設定sanitize、XR state遷移とcockpit rig座標を検証する。
smoke testはHTTP sampleとWebSocket経由のmanual/shared/auto telemetryを確認する。
