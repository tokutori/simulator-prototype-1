# Independent FDM evaluation

JSBSim 1.3.1をproduction dependencyにせず、独立reference implementationとして使う。
Rust simulationが記録した舵角時系列を、同じJSONから一時生成したJSBSim aircraftへ
open-loopでreplayし、plant同士のstate historyを比較する。

```powershell
python -m venv .eval-venv
& .\.eval-venv\Scripts\python.exe -m pip install -r evaluation\requirements.txt
cargo run -q -p sim-cli -- --model models\qx18-br-training-envelope.json --duration 5 --dt 0.01 --output reports\reference-input.csv
& .\.eval-venv\Scripts\python.exe evaluation\jsbsim_compare.py `
  --model models\qx18-br-training-envelope.json `
  --rust-csv reports\reference-input.csv `
  --output-csv reports\jsbsim-comparison.csv `
  --plot reports\jsbsim-comparison.png
```

比較は共通の係数databaseを別の運動方程式実装へ転記した**implementation verification**であり、
係数自体の実機妥当性は証明しない。またJSBSimの大気密度がJSON値と一致する高度を探索し、
絶対高度ではなく開始点からの高度変化を比較する。

## Controller parameter sweep

発進時のpitch-rate dampingとAoA protectionのtrade-offは36 caseの決定的sweepで評価する。

```powershell
cargo build -p sim-cli
& .\.eval-venv\Scripts\python.exe evaluation\control_sweep.py `
  --binary target\debug\sim-cli.exe `
  --model models\qx18-br-training-envelope.json `
  --output-csv reports\control-sweep.csv `
  --plot reports\control-sweep.png
```

## Launch scheduling sweep

pull-out開始/完了速度、launch AoA目標、pitch-rate look-aheadの135組を比較する。正の
flight-path sampleを許さず、高度損失、最大迎角、最深flight pathのtrade-offを見る。

```powershell
& .\.eval-venv\Scripts\python.exe evaluation\launch_strategy_sweep.py `
  --binary target\debug\sim-cli.exe `
  --model models\qx18-br-training-envelope.json `
  --output-csv reports\launch-strategy-sweep.csv `
  --plot reports\launch-strategy-sweep.png
```

## Longitudinal modes

`no_std` coreがtrim近傍の`[u, w, q, theta]` Jacobianを生成し、host CLIだけが
nalgebra 0.34.1で固有値を求める。coreへheapや数値解析dependencyを持ち込まない。

```powershell
cargo run -q -p sim-cli -- `
  --model models\qx18-br-training-envelope.json `
  --output reports\run.csv `
  --linearization-output reports\longitudinal-linearization.csv `
  --modes-plot reports\longitudinal-modes.png
```

## Deterministic robustness cases

公開再構成係数は実機同定値でないため、質量、`CL/CD/Cm`、`Cmq`、舵効き、servo lagを
one-at-a-timeと2つのcorner caseで振る。範囲は感度を見るengineering stressであり、
確率分布・許容公差・安全保証ではない。

```powershell
& .\.eval-venv\Scripts\python.exe evaluation\robustness_sweep.py `
  --binary target\debug\sim-cli.exe `
  --model models\qx18-br-training-envelope.json `
  --output-csv reports\robustness-sweep.csv `
  --plot reports\robustness-sweep.png
```

## Aerodynamic-envelope policy comparison

strict -5～8 deg modelは有効範囲外で停止させ、従来のendpoint保持とBR式を再構成した
訓練用-12～20 deg modelを同じ発進条件で比較する。これは三つのstall model比較ではない。
後二者はどちらもQX-18実機の高迎角dataではなく、境界処理による結果差を可視化する試験である。

```powershell
& .\.eval-venv\Scripts\python.exe evaluation\aero_envelope_compare.py `
  --binary target\debug\sim-cli.exe `
  --strict-model models\qx18-public-reconstruction.json `
  --training-model models\qx18-br-training-envelope.json `
  --output-csv reports\aero-envelope-comparison.csv `
  --plot reports\aero-envelope-comparison.png
```

## Ground effect / gust / wind sweep

free air、簡易ground effect、上下1−cos gust、向かい/追い/横風pulseを同じ発進条件で比較する。
gust magnitudeは大会気象の確率分布ではなくstress値である。

```powershell
& .\.eval-venv\Scripts\python.exe evaluation\environment_sweep.py `
  --binary target\debug\sim-cli.exe `
  --model models\qx18-br-training-envelope.json `
  --output-csv reports\environment-sweep.csv `
  --plot reports\environment-sweep.png
```

再浮上量だけへoverfitしないため、対地鉛直速度gain、filter時定数、glide pitch-rate dampingの
100組を、2 m/s上昇gustで再浮上量・最大上向き角・舵角飽和・総舵角変動として同時比較する。

```powershell
& .\.eval-venv\Scripts\python.exe evaluation\gust_controller_sweep.py `
  --binary target\release\sim-cli.exe `
  --model models\qx18-br-training-envelope.json `
  --output-csv reports\gust-controller-sweep.csv `
  --plot reports\gust-controller-sweep.png
```

32 Hz気圧高度から得る対地鉛直速度について、climb barrier、gain、filter時定数の60組を
nominalと2 m/s上昇gustで比較する。再浮上量、nominal高度、舵角総変動を別々に出し、
単一scoreで選択を隠さない。

```powershell
& .\.eval-venv\Scripts\python.exe evaluation\ground_path_barrier_sweep.py `
  --binary target\release\sim-cli.exe `
  --model models\qx18-br-training-envelope.json `
  --output-csv reports\ground-path-barrier-sweep.csv `
  --plot reports\ground-path-barrier-sweep.png
```

## RC / HPA flight-log holdout comparison

FRD/NED/SIへ変換済みの実測logとsimulation replayを補間比較する。時刻offsetやsensor biasは
validation対象なので自動fitしない。詳細は[`docs/flight-validation.md`](../docs/flight-validation.md)。

```powershell
& .\.eval-venv\Scripts\python.exe evaluation\flight_log_compare.py `
  --measured-csv reports\rc-holdout.csv `
  --simulation-csv reports\rc-holdout-simulation.csv `
  --output-csv reports\rc-holdout-residual.csv `
  --metrics-json reports\rc-holdout-metrics.json `
  --plot reports\rc-holdout.png
```

## Actual-UF2 sensor fault injection

virtual BNO055/AS5600/SDP810/DPS310へ、plant時刻またはfirmware control-update数で
決定的なstatus/CRC faultを入れる。control-critical sensorは1～2 updateでhold-last、3回目に
model指定固定failsafe舵、20 valid update後に再armする。SDP810単独faultは最後のvalid
airspeedを保持するdegraded modeとし、GPIO18のraw-invalidとGPIO17のfailsafeを分離して確認する。
加速したrp2040jsのwall/plant時間は性能値に使わず、状態遷移はcontrol-update差分で評価する。

```powershell
npm.cmd run simulate --prefix virtual-platform -- --steps 700 --timing-acceleration 50 `
  --sensor-fault bno-status --fault-start-s 3 --fault-duration-s 0 `
  --fault-update-count 3 --output reports\fault-update-bno-three.csv `
  --summary reports\fault-update-bno-three.json
& .\.eval-venv\Scripts\python.exe evaluation\fault_injection_plot.py `
  --reports reports --plot reports\fault-injection.png
```

永続sensor loss時の固定舵角をhold-lastおよび0～3 deg（正=機首下げ）と比較する。
接水を含めて評価し、再浮上量だけを最適化しない。

```powershell
& .\.eval-venv\Scripts\python.exe evaluation\failsafe_command_sweep.py `
  --output-csv reports\failsafe-command-sweep.csv `
  --plot reports\failsafe-command-sweep.png
& .\.eval-venv\Scripts\python.exe evaluation\persistent_fault_plot.py `
  --reports reports --plot reports\persistent-fault-response.png
```

最小余裕だった2.0秒故障を、既存の17個のdeterministic model-uncertainty caseと
+0.75/+1.0/+1.5 degで交差比較する。範囲は同定分布ではなく反証用stressである。

```powershell
& .\.eval-venv\Scripts\python.exe evaluation\failsafe_robustness_sweep.py `
  --output-csv reports\failsafe-robustness-sweep.csv `
  --plot reports\failsafe-robustness-sweep.png
```

SDP810だけの喪失では最後のvalid airspeedを保持し、IMU/AoA/barometerによるcontrollerを継続する
degraded modeをactual UF2と17 model caseで反証する。failsafe GPIOとは別にraw-invalid GPIOを維持する。

```powershell
& .\.eval-venv\Scripts\python.exe evaluation\degraded_airspeed_robustness.py `
  --fault-start-s 0.5 `
  --output-csv reports\degraded-airspeed-early-robustness.csv `
  --plot reports\degraded-airspeed-early-robustness.png `
  --log-directory reports\degraded-airspeed-early-logs
```

`--sensor-fault none`では同じactual UF2をfaultなしで17 model caseへ掛けられる。
degraded制御は0.25 s経路先読みとreleaseから0.75～2.0 mの相対気圧高度blendを使うが、
閾値は再構成nominal軌道由来であり、実flightの確率分布ではない。
