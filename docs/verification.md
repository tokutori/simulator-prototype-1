# Verification status

確認日は2026-08-25。ここでは「数式実装が意図したmodelを解いているか」と
「そのmodelが実機を表すか」を分ける。

## Analytic checks

`flight-dynamics-core::steady_glide_trim`は、table内で`Cm = 0`となる迎角をbisectionで求め、
無推力のforce balanceから速度と飛行経路角を計算する。QX-18 sampleの無操舵解は次である。

| quantity | result |
| --- | ---: |
| airspeed | 9.698 m/s |
| alpha | 1.682 deg |
| flight path | -1.625 deg |
| pitch | 0.057 deg |
| L/D | 35.2 |

公開BR Simulatorから再構成した9.7 m/s trimと一致する。この値はunit testで固定し、
table、density、mass、reference areaの意図しない変更を検出する。

## Longitudinal linearization

`flight-dynamics-core::linearize_steady_glide`は非線形plantをsteady glideまわりで中央差分し、
`delta_x_dot = A delta_x + B delta_e`、`x = [u, w, q, theta]`を固定長配列で返す。
trim residualは`u_dot = 5.38e-17 m/s^2`、`w_dot = -3.55e-15 m/s^2`、
`q_dot = -4.38e-17 rad/s^2`で、数値丸めの範囲に収まった。host側だけで
[nalgebra 0.34.1](https://docs.rs/crate/nalgebra/0.34.1)の実Schur分解を使った結果は次の通り。

| pair | eigenvalues [1/s] | natural frequency | damping ratio | period | decay time constant |
| --- | --- | ---: | ---: | ---: | ---: |
| fast | -7.4001 ± 3.5944j | 8.2268 rad/s | 0.8995 | 1.748 s | 0.135 s |
| slow | -0.09070 ± 0.61067j | 0.61737 rad/s | 0.1469 | 10.289 s | 11.025 s |

この再構成modelでは両modeともopen-loop stableだが、slow pairは弱減衰である。これは
実機同定結果ではなく、主に公開`Cm(alpha)`、`Cmq`、慣性momentから導かれたmodel propertyである。
controller設計ではfast pair、servo 0.06 s、IMU 100 Hzを別々に無視しない。
matrixは`reports/longitudinal-linearization.csv`、graphは`reports/longitudinal-modes.png`へ出す。

## Independent JSBSim comparison

[JSBSim 1.3.1](https://github.com/JSBSim-Team/jsbsim)のPython APIへ、model JSONと同じ
mass、inertia、geometry、`CL/CD/Cm(alpha)`、`CL_delta_e`、`Cm_q`、`Cm_delta_e`を転記した。
Rust closed-loop runが記録したelevator時系列をJSBSimへopen-loop replayし、5秒間比較した。
大気密度はJSBSim側の高度を探索して1.164 kg/m³へ一致させ、terrain elevationをずらして
Rustと同じAGLを与えた。簡易ground-effect誘導抗力補正も独立XML式へ転記した。

| state | RMS difference | maximum absolute difference |
| --- | ---: | ---: |
| altitude change | 0.0208 m | 0.0346 m |
| airspeed | 0.0139 m/s | 0.0243 m/s |
| alpha | 0.0636 deg | 0.1872 deg |
| pitch | 0.1331 deg | 0.2633 deg |
| flight path | 0.1063 deg | 0.2266 deg |
| pitch rate | 0.00435 rad/s | 0.0216 rad/s |

独立実装がほぼ同じ軌道を返すため、現在の大きな発進降下をRustの座標変換やRK4固有の
誤りだけで説明する証拠はない。ただし同じ係数databaseを使うため、係数の誤りには両者が
同時に一致してしまう。これはimplementation verificationであってmodel validationではない。

実行scriptは[`evaluation/jsbsim_compare.py`](../evaluation/jsbsim_compare.py)、比較graphは
`reports/jsbsim-comparison.png`へ生成する。

## Controller sweep

発進時pitch-rate damping 6水準 × AoA protection gain 6水準、計36 caseを8秒ずつ実行した。
全caseで正の飛行経路角sampleと再浮上は0だったが、-3 degへ回復するまでの高度損失は
4.78～6.72 mだった。現行値は5.01 mで、最小値との差は0.23 mに過ぎない。最小caseは
最大迎角が17.13 degと現行の16.98 degより悪化するため、graphだけを良くするgain変更は
採用しなかった。

この結果は、2 m程度の降下後に引き起こす公開説明との差をcontroller gainだけで解決せず、
次のdataを優先すべきことを示す。

- 8 degを超える`CL/CD/Cm(alpha)`とdynamic stall/unsteady response
- 実機または縮尺RCの発進直後pitch/alpha/airspeed/elevator log
- actuator負荷時responseと実舵角
- wing/tail-boomの主要構造mode

実行scriptは[`evaluation/control_sweep.py`](../evaluation/control_sweep.py)、heatmapは
`reports/control-sweep.png`へ生成する。

pull-out開始/完了速度、launch AoA目標、look-aheadの135組も別に探索した。全caseで正の
飛行経路角は0だった。従来の7.0～8.5 m/s、0.15 s先読みから6.0～7.5 m/s、0.10 sへ
前倒しすると、最大迎角16.98 degを維持したまま高度損失を5.57 mから5.01 mへ減らせた。
最小損失caseは5.00 mだが最大迎角17.22 degとなるため採用しなかった。結果は
[`evaluation/launch_strategy_sweep.py`](../evaluation/launch_strategy_sweep.py)と
`reports/launch-strategy-sweep.png`へ出力する。

## Deterministic uncertainty stress cases

未同定modelを一点値として扱わないため、質量±5%、`CL`±10%、`CD`±20%、
`Cm(alpha)`±20%、`Cmq`±30%、elevator effectiveness±20%、servo時定数0.03～0.12 sを
one-at-a-timeで変え、さらにadverse/favorable cornerを実行した。これらは公開公差ではなく、
感度確認用の明示的engineering rangeである。

| result | baseline | range across 17 cases | adverse corner |
| --- | ---: | ---: | ---: |
| altitude loss until -3 deg recovery | 5.01 m | 3.57–7.37 m | 7.37 m |
| steepest downward flight path | -23.93 deg | -28.87～-19.35 deg | -28.87 deg |
| maximum alpha | 16.98 deg | 14.09–19.86 deg | 19.86 deg |
| time outside training aero table | 0 s | 0 s in every case | 0 s |
| maximum re-ascent | 0 m | 0 m in every case | 0 m |

再浮上防止logicは全caseで目的を満たした。一方、訓練用tableは公開BR式を20 degまで
再構成しただけで、実測失速dataではない。adverse cornerは19.86 degと上限まで0.14 degしか
余裕がないため、高度損失の絶対値はvalidation結果ではない。ここでも最優先課題はcontroller
再調整より実測nonlinear aero dataである。結果は
[`evaluation/robustness_sweep.py`](../evaluation/robustness_sweep.py)、
`reports/robustness-sweep.csv`、`reports/robustness-sweep.png`へ生成する。

## Ground effect and deterministic gust stress

ground effectはBR Simulatorと同形の`h/b` correlationを誘導抗力成分だけに適用した。
free-airと比較すると20秒時点の高度は0.836 mから0.912 mとなり、nominal再浮上はどちらも0。
これは約7.2 cmの差を予測精度として主張する結果ではなく、簡易式が限定的な方向へ作用し、
controller回帰を壊さないことの確認である。

距離60～100 mにfull 1−cos wind pulseを与えた。上昇gustで`pitch-alpha`だけを使う旧制御は、
0.5/1/2 m/sで0.312/1.176/2.972 m再上昇した。32 Hz sample-and-hold気圧高度が変化した時だけ
経過時間で差分し、sample間は推定値を保持するよう修正した。0.25 s filter、
`-0.25 m/s`の対地barrier、0.9 rad/(m/s) feedback、回復後0.6 s pitch-rate dampingでは
0/0.027/0.157 mとなった。一方、下降gust 0.5/1/2 m/sでは19.71/15.84/9.29 sで接水した。
従って外乱時の再上昇0を
保証せず、再上昇量、接水時間、舵角飽和、迎角marginを同時に評価する。

対地barrier 5水準、gain 3水準、filter 4水準の60 caseを2 m/s上昇gustで探索した。
現行`-0.25 m/s`、0.9、0.25 sはhost上で再浮上0.157 m、gust舵角総変動102.8 deg、
nominal舵角総変動68.2 degだった。filter 0.10 sは再浮上0.070 mへ減るが、gust/nominalの
総変動を112.6/78.9 degへ増やすため採用しなかった。発進中に高いdampingを掛けると通常軌道の
高度損失が増えたため、7.5 m/s以上かつ-3 degまで回復した時点でglide dampingをlatchし、
0.25 sで移行する。これは実機用gainの決定ではなく、未validation plantに対するstress設定である。

現在のgustは機体全体一様のfrozen fieldで、unsteady lift、spanwise gradient、flexibilityを
含まない。ground effectもlift/downwash/momentを変えず、planform依存を同定していない。
結果は[`evaluation/environment_sweep.py`](../evaluation/environment_sweep.py)、
`reports/environment-sweep.csv`、`reports/environment-sweep.png`へ生成する。

## Flight-log comparison plumbing

RC/HPA実測logを共通FRD/NED/SI contractへ変換した後、simulationを実測時刻へ補間して
bias、MAE、RMSE、最大絶対誤差を出すtoolを追加した。同一`run.csv`同士のself-checkは
全state誤差0。これは比較配管だけの検査で、実機dataによるmodel validationではない。
時刻offsetとsensor biasを自動fitしない理由、同定maneuverとholdout maneuverの分離は
[`flight-validation.md`](flight-validation.md)に記した。

## Aerodynamic-envelope boundary

同じ5 m/s、下向き3 degの発進条件に三つのboundary処理を与えた。

| case | result | maximum alpha | pull-out loss to -3 deg |
| --- | --- | ---: | ---: |
| strict -5～8 deg | 0.10 sで`aero-envelope-exit` | 8.45 deg | 未算出 |
| legacy 8 deg endpoint hold | 8 s完走、0.92 s範囲外 | 18.98 deg | 4.52 m |
| BR training -12～20 deg | 8 s完走、範囲外0 | 16.98 deg | 5.01 m |

endpoint保持はBR訓練用再構成より0.48 m少ない高度損失を返し、最大迎角を2.00 deg大きく
見積もった。従来の見た目は根拠のないtable外処理へ依存していたため、defaultから外した。
ただしBR側も実測stall modelではない。比較graphは`reports/aero-envelope-comparison.png`。

## Actual-UF2 virtual-platform comparison

host adapterとRP2040 firmwareは同じ`fbw-control-core`を使うが、後者はBNO055等のregister
量子化、I²C transaction、SDP CRC、servo PWMの経路を通る。20秒、2000 sampleを比較した。

| quantity | RP2040 UF2 vs native host |
| --- | ---: |
| altitude RMSE | 0.0290 m |
| final altitude difference | -0.0476 m |
| flight-path RMSE | 0.0513 deg |
| final flight-path difference | -0.0065 deg |
| actual elevator RMSE | 0.267 deg |
| UF2 case maximum re-ascent | 0 m |
| UF2 positive flight-path samples | 0 |

差は主にf64→f32、sensor/protocol量子化、PWM pulseの1 us量子化から生じる。両経路とも同じ
FDMと係数を使うので、これはfirmware integration verificationであり実機model validation
ではない。比較graphは`reports/virtual-platform-comparison.png`へ生成する。

MCU instruction timeは50倍加速しており、`timing_validated=false`をsummaryへ必ず出す。
deadline、interrupt jitter、brownout、servo電流、配線/level shiftingはphysical HILで評価する。

actual UF2の0.5/1/2 m/s上昇gustでは再浮上0/0.029/0.186 m、最大飛行経路角
-0.43/0.50/2.10 degだった。gust区間の最大実舵角は3.11/5.46/9.77 degで、
2 m/s caseの余裕は約0.23 degである。従って「再浮上しない」ことは
nominal acceptanceであり、強い外乱に対する保証ではない。

## Actual-UF2 sensor fault and timing checks

firmwareのBNO055 chip/system status、AS5600 magnet status、SDP810全word CRC、DPS310
sensor/coefficient readyをvirtual deviceとproduction driverの両側に実装した。共有`no_std`
安全gateは10 valid updateでarm、2 invalid updateまでhold-last、3回目にmodel contract指定の
固定failsafe舵、20 valid updateで再armする。QX-18訓練profileは正を機首下げとして+0.75 degである。
SDP810単独faultは最後のvalid airspeedを保持するdegraded inputとして扱い、GPIO18で異常を示しながら
IMU/AoA/barometerによるcontrolを継続する。

DPS310 datasheetの`PRS_RDY`は新pressure resultを示し、pressure register readでclearされる。
virtual deviceは32 Hzでbitを生成してpressure 3 byte readでclearし、firmwareはbitが0の100 Hz周期で
直前pressureを最大20 read保持し、それを超えると異常にする。controllerもbarometric値が変化した
時だけ鉛直速度を更新してsample-and-holdの誤微分を防ぐ。40 updateの`dps-stale`固定試験では
開始0.23 s後、invalid 3回目でfailsafeへ入り、20 valid updateで復帰した。

| actual-UF2 case（t=3 s） | failsafe回数 | invalid→failsafe | recovery valid→rearm | 再浮上 | 7 s高度差対nominal |
| --- | ---: | ---: | ---: | ---: | ---: |
| SDP CRC 1 update | 0 | - | - | 0 m | 0 m |
| SDP CRC 3 update | 0 | - | - | 0 m | 0 m |
| SDP I²C NACK 3 update | 0 | - | - | 0 m | +0.0025 m |
| BNO status 3 update | 1 | 2 update差 | 19 update差 | 0 m | +0.0116 m |
| AS5600 magnet 3 update | 1 | 2 update差 | 19 update差 | 0 m | +0.0116 m |
| DPS ready 3 update | 1 | 2 update差 | 19 update差 | 0 m | +0.0116 m |

差分は「3回目」「20回目」を表す。状態遷移はGPIO17～19でfirmware control updateに同期して
観測した。固定舵はこの仮モデルで小さい高度差に留まっただけで、安全証明ではない。
比較graphは`reports/fault-injection.png`。

全control inputを失う場合のfailsafe commandを、0.5/1/2/3/5/7秒の開始時刻で12秒までsweepした。

| strategy | 6 case中の最大再浮上 | 接水case数 | 最大飛行経路角 |
| --- | ---: | ---: | ---: |
| hold-last | 3.374 m | 0 | 18.91 deg |
| fixed 0.00 deg | 5.607 m | 0 | 11.05 deg |
| fixed +0.50 deg | 0.042 m | 2 | 0.22 deg |
| fixed +0.75 deg | 0 m | 2 | -0.23 deg |
| fixed +1.00 deg | 0 m | 2 | -0.38 deg |
| fixed +1.50 deg | 0 m | 2 | -0.21 deg |

nominalで試した最小介入の+0.75 degを暫定値にした。しかし2秒faultを17個のmodel stressへ広げると、
+0.75/+1.0/+1.5 degは7/7/10 caseで再浮上し、最悪0.920/0.679/0.251 mだった。
固定舵を強めるだけではrobustにならない。actual UF2の永続BNO status faultはnominal 6時刻で
再浮上0だが、0.5/1秒開始は3.37/3.66秒で接水し、2秒開始の最大経路角は-0.055 deg。
比較graphは`reports/failsafe-command-sweep.png`と`reports/failsafe-robustness-sweep.png`、
actual-UF2 summaryは`reports/failsafe-bno-start-*.json`、時系列は`reports/persistent-fault-response.png`。

SDP単独faultは最後のvalid airspeedを保持してcontrolを継続する。actual UF2の永続NACKでは、
0.25 s経路先読みとreleaseから0.75～2.0 mの相対気圧高度blendにより、0.5/1/2/3/5/7秒の
全開始時刻で再浮上0・正経路角sample 0・接水0となった。0.5秒caseの最大経路角は-1.37 deg、
command/actual舵角総変動93.4/60.0 deg。時系列は`reports/degraded-airspeed-timing.png`。
0.5秒開始を17個のmodel stressへ掛けても全caseで再浮上0・接水0、failsafe 0、deadline miss 0。
adverse cornerの最大経路角は-1.04 deg、12秒高度は0.278 mだった。
`reports/degraded-airspeed-early-robustness.png`に示す。閾値は実機同定値でなく再構成nominal軌道から
選んだ工学値である。

従来のloop末尾10 ms delayは処理時間を周期へ上乗せしていたため、1 MHz timerで処理時間を
差し引く固定周期へ修正した。加速率sweepは次の通り。

| rp2040js加速率 | 3 sのcontrol update | deadline miss観測 | 最終高度 |
| ---: | ---: | ---: | ---: |
| 10 | 302 | 0 | 5.275171 m |
| 20 | 302 | 0 | 5.275171 m |
| 50 | 302 | 0 | 5.275160 m |
| 100 | 241 | 3回、2.95 s high | 5.259872 m |
| 200 | 121 | 2回、2.96 s high | 5.220297 m |
| 500 | 51 | 1回、2.99 s high | 5.180420 m |

従ってdefaultは50倍とした。10～50倍の一致はvirtual schedulerの回帰根拠にはなるが、
実MCUのdeadline保証にはならず、`timing_validated=false`を維持する。

## Dependency and schema checks

`cargo audit`で既知vulnerabilityは0件だった。unmaintained warningは`nalgebra 0.34.1`経由の
`paste 1.0.15`と、`plotters 0.3.7`経由の`ttf-parser 0.20.0`の2件。直接dependencyでは
ないため、upstream更新を追跡する。model contractはAjv CLIの`--spec=draft2020`で三modelを
検証した。Ajvの既定draftでは2020-12 meta-schemaを解決できないため、spec指定を省略しない。

firmware側lockfileも既知vulnerabilityは0件。unmaintained warningは`cortex-m`系の
`bare-metal 0.2.5`、`rp2040-hal`系の`paste 1.0.15`と`proc-macro-error2 2.0.1`である。
直接dependencyへ無理に置換せず、HAL/cortex-mの互換更新を追跡する。npm auditは0件だった。

## Reproduction

```powershell
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo check -p flight-dynamics-core --target thumbv6m-none-eabi
npx.cmd --yes ajv-cli validate --spec=draft2020 -s models\model-contract.schema.json -d models\qx18-public-reconstruction.json -d models\qx18-br-training-envelope.json -d models\illustrative-hpa.json --strict=false

python -m venv .eval-venv
& .\.eval-venv\Scripts\python.exe -m pip install -r evaluation\requirements.txt
cargo run -q -p sim-cli -- --model models\qx18-br-training-envelope.json --duration 5 --dt 0.01 --output reports\reference-input.csv
& .\.eval-venv\Scripts\python.exe evaluation\jsbsim_compare.py --model models\qx18-br-training-envelope.json --rust-csv reports\reference-input.csv --output-csv reports\jsbsim-comparison.csv --plot reports\jsbsim-comparison.png
cargo run -q -p sim-cli -- --model models\qx18-br-training-envelope.json --output reports\run.csv --linearization-output reports\longitudinal-linearization.csv --modes-plot reports\longitudinal-modes.png
cargo build -p sim-cli
& .\.eval-venv\Scripts\python.exe evaluation\robustness_sweep.py --binary target\debug\sim-cli.exe --model models\qx18-br-training-envelope.json --output-csv reports\robustness-sweep.csv --plot reports\robustness-sweep.png
& .\.eval-venv\Scripts\python.exe evaluation\launch_strategy_sweep.py --binary target\debug\sim-cli.exe --model models\qx18-br-training-envelope.json --output-csv reports\launch-strategy-sweep.csv --plot reports\launch-strategy-sweep.png
& .\.eval-venv\Scripts\python.exe evaluation\aero_envelope_compare.py --binary target\debug\sim-cli.exe --strict-model models\qx18-public-reconstruction.json --training-model models\qx18-br-training-envelope.json --output-csv reports\aero-envelope-comparison.csv --plot reports\aero-envelope-comparison.png
& .\.eval-venv\Scripts\python.exe evaluation\environment_sweep.py --binary target\debug\sim-cli.exe --model models\qx18-br-training-envelope.json --output-csv reports\environment-sweep.csv --plot reports\environment-sweep.png
& .\.eval-venv\Scripts\python.exe evaluation\flight_log_compare.py --measured-csv reports\run.csv --simulation-csv reports\run.csv --output-csv reports\flight-log-self-check.csv --metrics-json reports\flight-log-self-check.json --plot reports\flight-log-self-check.png --relative-columns altitude_m
cargo build --manifest-path firmware\fbw-rp2040\Cargo.toml --target thumbv6m-none-eabi --release
New-Item -ItemType Directory -Force target\virtual-platform | Out-Null
elf2uf2-rs firmware\fbw-rp2040\target\thumbv6m-none-eabi\release\fbw-rp2040 target\virtual-platform\fbw-rp2040.uf2
cargo build -p sim-cli --bin plant-bridge
npm.cmd install --prefix virtual-platform
npm.cmd run check --prefix virtual-platform
npm.cmd test --prefix virtual-platform
npm.cmd audit --prefix virtual-platform --audit-level=high
cargo run -q -p sim-cli --bin sim-cli -- --model models\qx18-br-training-envelope.json --duration 20 --dt 0.01 --output reports\native-shared-controller.csv
npm.cmd run simulate --prefix virtual-platform -- --steps 2000 --timing-acceleration 50 --output reports\virtual-platform.csv
& .\.eval-venv\Scripts\python.exe evaluation\virtual_platform_compare.py --host reports\native-shared-controller.csv --virtual reports\virtual-platform.csv --plot reports\virtual-platform-comparison.png --summary reports\virtual-platform-summary.json
npm.cmd run simulate --prefix virtual-platform -- --steps 700 --sensor-fault sdp-crc --fault-start-s 3 --fault-duration-s 0 --fault-update-count 3 --output reports\fault-update-sdp-three.csv --summary reports\fault-update-sdp-three.json
& .\.eval-venv\Scripts\python.exe evaluation\fault_injection_plot.py --reports reports --plot reports\fault-injection.png
```
