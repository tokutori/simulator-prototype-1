# Birdman Flight Dynamics Simulator Prototype 1

鳥人間向けFBW検証環境の最初の実行可能prototypeです。platform-independentな
`#![no_std]` Rust coreと、JSON/CSVを扱うhost CLIを分離しています。

default sampleは公開QX-18 dataとBR Simulatorの式を再構成した**訓練用・未validation
high-alpha model**です。実在機の公式諸元、公開simulatorの推定空力値、実搭載例と部品
datasheetを区別して追跡します。実機挙動、飛距離、stall safetyを予測するflight-test
同定modelではありません。

現行の修正状況・検証結果と未検証境界は
[全体レビューの追跡記録](docs/full-review-followup.md)を参照してください。
機体入力はschema 0.12（対地／対気発進速度、空力の座標系、CGモーメント基準を明示）です。
電装の対応範囲は[BNO055取付契約](docs/bno055-frame-contract.md)、
[UART送信契約](docs/uart-recorder-contract.md)、
[記録・終了理由の契約](docs/experiment-recording.md)に分けて記載しています。

## 実装済み

- body Forward-Right-Down（FRD）、navigation North-East-Down（NED）、SI units
- quaternion姿勢、body velocity、body angular rateによるnonlinear rigid 6DoF
- fixed-step RK4
- table内の`Cm = 0`とforce balanceを解く無推力steady-glide trim solver
- trim近傍のallocation-free 4-state縦運動linearization
- table-driven `CL/CD/Cm(alpha)`、modelごとのtable外停止またはclamp-and-flag policy
- beta/rate/control derivativeによる`CY/Cl/Cn`
- `Ixz`を含むsymmetric inertia tensor
- elevator/rudderのtravel、rate、first-order lag、deadband、command量子化
- BNO055相当100 Hz IMU sample-and-hold・register量子化
- SDP810-500Pa相当のrange・pressure量子化・responseと100 Hz polling、DPS310相当32 Hz barometer、AS5600相当100 Hz AoA
- strict JSON model loaderとCSV logger
- native reference energy-management controllerによるclosed-loop smoke test
- hostとRP2040 firmwareで同じ`#![no_std]` `fbw-control-core`を使用
- 共有`#![no_std]` safety gateによるhold-last、model指定固定failsafe舵、rearm
- critical sensorの4連続read失敗後に、離陸時気圧基準を保持してdeviceを再初期化する状態機械
- 5 m/s補助発進caseのAoA→飛行経路角smooth transition、pitch-rate先読み、nominal再浮上防止
- 32 Hz気圧高度のdistinct sampleだけを差分する対地鉛直速度filter、sink-rate barrier、glide damping
- 誘導抗力への簡易ground-effect correlationと距離領域1−cos gust
- Plotters 0.3.7によるflight trajectory、pitch/alpha/flight-path、airspeed、舵角のPNG出力
- 空力table範囲外をCSV、summary、PNGの赤点で明示
- 最大飛行経路角、最低高度後の再浮上量、正経路角sample数をsummaryと回帰testで監視
- nalgebra 0.34.1によるhost側固有mode解析と固有値PNG
- JSBSim 1.3.1への同一舵角replayによる独立FDM比較
- controller parameter sweepと決定的model-uncertainty stress cases
- ground effect/gust/windの環境sweepとRC/実機log比較tool
- 実RP2040向けELF/UF2、`rp2040-hal` MMIO、`rp2040js` virtual I2C/PWMによるclosed loop
- BNO055、AS5600、SDP810、DPS310のdatasheet-level register protocolとSDP CRC
- status/CRC/I²C NACKの決定的fault injectionとactual-UF2永続故障sweep
- SDP810単独喪失時のheld-airspeed、相対気圧高度pull-outを使うdegraded controlと17 model stress比較
- Three.jsによる一人称/追従三人称replay、動く波面、start中心距離ring、HUD、CSV読込み
- keyboard/gamepadの設定可能なelevator/rudder入力とmanual/shared/auto連続authority比較
- 標準120秒（nominal 10 m/sで約1.2 km）の記録枠、着水・空力範囲逸脱時の早期終了
- 別tabの飛行後analysis（軌跡、高度、速度、pilot入力、command/実舵角、姿勢）

## 実行

Rust 1.88.0を使用します。

```powershell
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo run -p sim-cli -- --model models/qx18-br-training-envelope.json `
  --output reports/run.csv --plot reports/run.png `
  --linearization-output reports/longitudinal-linearization.csv `
  --modes-plot reports/longitudinal-modes.png
```

RP2040 virtual platformを含む20秒run:

```powershell
cargo build --manifest-path firmware\fbw-rp2040\Cargo.toml `
  --target thumbv6m-none-eabi --release
New-Item -ItemType Directory -Force target\virtual-platform | Out-Null
elf2uf2-rs firmware\fbw-rp2040\target\thumbv6m-none-eabi\release\fbw-rp2040 `
  target\virtual-platform\fbw-rp2040.uf2
cargo build -p sim-cli --bin plant-bridge
npm.cmd install --prefix virtual-platform
npm.cmd run check --prefix virtual-platform
npm.cmd run simulate --prefix virtual-platform -- `
  --steps 2000 --timing-acceleration 1 --output reports\virtual-platform.csv
```

firmwareは10 ms固定周期を目標に実センサ互換I²C transactionを行い、KRS-4034HVの許容範囲内にある
20 ms周期・1000～2000 usのPWMを出す。通常実行はCPU時間倍率1固定（batchも既定1）。
倍率変更はCPUと周辺機器の時間関係を変えるため、任意の高速化ではなく明示的な負荷実験である。
実時間に追いつかないPCではWeb画面に警告する。倍率1でも実機のcycle精度を保証しない。
最新の境界・修正・検証状況は[全体レビュー修正記録](docs/review-remediation.md)を参照。

CSVを標準出力へ出す場合:

```powershell
cargo run -p sim-cli -- --duration 5 --dt 0.01
```

一人称/三人称viewerとinteractive pilot-in-the-loop:

```powershell
cd visualizer-web
npm.cmd install
npm.cmd install --prefix ..\virtual-platform
npm.cmd run sample
npm.cmd run dev
```

`http://127.0.0.1:4173/`を開く。defaultは`Interactive`で、production RP2040 UF2を
rp2040jsで実行するlive loopへ直ちに入る。keyboard 4 key、gamepad stick、gamepad buttonを
elevator/rudderごとに選択でき、Auto authorityを0～100%で変更できる。対応browser/headsetでは
任意にWebXR cockpitへ入れる。詳細と制約は
[interactive visualizer](docs/interactive-visualizer.md)を参照する。

CLIの標準記録上限は120秒であり、nominal 10 m/sなら約1.2 kmを収容する。これは飛距離を
1 kmへ強制する設定ではなく、着水または空力table範囲逸脱時にはその時点で記録を終了する。
標準sample機は現在のmodel/controlでは約236 mで着水する。viewerの`Flight analysis`から、
現在のreplayまたはinteractive flightを別tabで時系列解析できる。

## 構成

```text
crates/
  fbw-control-core/        host/firmware共有、no_std f32 controller
  fbw-safety-core/         host/firmware共有、no_std sensor-validity gate
  flight-dynamics-core/  no_std、allocation-free、I/Oなしのphysics core
  sim-cli/               JSON、CLI、CSVというhost adapter
firmware/fbw-rp2040/      実target向けno_std firmware
virtual-platform/         rp2040js、virtual I2C devices、Rust plant bridge
visualizer-web/           Three.js replay、interactive input、local Rust bridge server
models/
  model-contract.schema.json
  qx18-public-reconstruction.json 出典付きの-5～8 deg strict model
  qx18-br-training-envelope.json  BR式を-12～20 degで再構成したdefault training model
  illustrative-hpa.json            架空のsoftware-test用model
docs/
  architecture.md
  qx18-and-avionics-model.md
  verification.md
  flight-validation.md
  interactive-visualizer.md
  post-flight-analysis.md
  ui-scaling.md
  completion-audit.md
  roadmap.md
```

設計根拠と公開事例の検証結果は隣接repository
[`simulator-search`](../simulator-search/README.md)にあります。

## `embedded-rust-playground`との関係

参照projectの次の境界を維持しています。

```text
virtual plant -> virtual sensor/peripheral -> actual target firmware
               actual target firmware -> actuator output -> virtual plant
```

同じ境界を実装した。firmwareにsimulator専用sensorやconditional mockはなく、実
`rp2040-hal`がI²C0とPWM0のMMIOを操作し、`rp2040js`がBNO055、AS5600、SDP810、DPS310
互換transactionを受ける。FDMはTypeScriptへ移植せず、NDJSONの`plant-bridge`を介して
同じRust physics coreを一stepずつ進める。

interactive viewerのlive modeもRust FDMを使用するが、pilot input hardwareが未選定のため
browserのkeyboard/gamepadをvirtual ADC/GPIOへ割り当てる。control/sensor/mixing/PWMは
production UF2をrp2040jsで実行し、host controllerへfallbackしない。host側はRust servo/FDM
plantだけを担当する。Web UIはTEAの判別可能unionでbackend状態を管理し、rp2040jsがwall-clockへ
追従できない場合は画面に明示する。

## 開発方針との対応

[プロジェクトの開発方針](https://zenn.dev/bem130/articles/1b352797de94e7)に従い、
coreは`no_std`、filesystem/network/clock非依存、明示入力・明示出力にしています。
errorはenum、modelはstruct、platform依存処理はCLIへ限定し、prototypeでも公開境界を
暫定的な雑設計にしない方針です。

## 制約

現在のground effectは誘導抗力だけの未validation簡易式、gustは機体全体一様の決定的pulseです。
launch contact dynamics、spanwise gust、post-stall lift drop/hysteresis、unsteady aerodynamics、
aeroelasticity、stochastic noise/fault、電気故障、実時間deadline、flight-qualified firmwareは扱いません。
決定的status/CRC/NACK faultは実装済みですが、実故障率や共通原因故障を表しません。
dataの根拠は[QX-18と電装model](docs/qx18-and-avionics-model.md)、残課題は
[roadmap](docs/roadmap.md)を参照してください。

## License

MIT License. Copyright (c) 2026 Bem130.
