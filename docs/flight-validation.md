# RC・実機flight log validation

このworkflowは「6DoF engineが正しいか」と「対象機の係数が正しいか」を分ける。
縮尺RC機は座標変換、sensor生成、actuator interface、log処理のengine validationには使えるが、
Reynolds数、翼面荷重、慣性分布、柔軟性が異なる鳥人間機modelの代用にはしない。

## 共通CSV contract

実測logger固有のraw logはadapterで次のSI/FRD/NED列へ変換し、raw fileも保存する。

| column | contract |
| --- | --- |
| `time_s` | 共通hardware triggerからの単調増加時刻 |
| `north_m`, `east_m`, `altitude_m` | NED位置から得る対地位置・高度 |
| `roll_deg`, `pitch_deg`, `yaw_deg` | body-to-NED姿勢 |
| `p_rad_s`, `q_rad_s`, `r_rad_s` | body FRD角速度 |
| `airspeed_mps`, `alpha_deg` | 校正済みair data。欠測は補間せず別flagを持つ |
| `elevator_deg`, `rudder_deg` | **実舵角**。command値だけで代用しない |
| command columns | firmware出力を別列で保存し、servo遅延同定に使う |

座標変換、時刻offset、filter、欠測処理、sensor firmware version、機体configuration、CG、
mass、風をmanifestに残す。比較scriptは時刻やbiasを自動最適化しない。そこを自動調整すると、
FBWで重要なtransport delayや零点誤差まで見えなくなるためである。

## maneuverと分離検証

安全管理された試験環境で、機体・操縦者に適した小さな入力から段階的に試験する。
具体的な舵角や高度は機体強度、運用規程、試験責任者の判断なしに固定しない。

1. 静止地上試験でaxis、sign、sample rate、時刻同期、実舵角sensorを確認する。
2. 定常区間でbiasとtrimを記録する。
3. elevator/rudderのdoubletまたはstep-like inputから短周期応答とservo responseを得る。
4. 同定用maneuverで係数・遅延をfitする。
5. **別のholdout maneuver**を一切再調整せずreplayし、time historyの誤差を評価する。
6. RCでengine/workflowを確認後、鳥人間実機TFのlogで対象aircraft modelを更新する。

NASAのLight Eagle/Daedalus試験と同様、入力と応答を時系列で比較し、rigid-body係数だけで
残差を説明できない場合はwing bending、tail-boom bending、unsteady aeroを候補にする。

## 比較tool

adapter済み実測CSVと、同じ実舵角履歴を与えたsimulation CSVを比較する。

```powershell
& .\.eval-venv\Scripts\python.exe evaluation\flight_log_compare.py `
  --measured-csv reports\rc-holdout.csv `
  --simulation-csv reports\rc-holdout-simulation.csv `
  --output-csv reports\rc-holdout-residual.csv `
  --metrics-json reports\rc-holdout-metrics.json `
  --plot reports\rc-holdout.png `
  --relative-columns altitude_m
```

出力は各stateのbias、MAE、RMSE、最大絶対誤差と全sample residualである。許容値は用途別に
事前登録し、観測後に合格線を動かさない。現在のscriptは比較・可視化までで、parameter fit、
logger adapter、実舵角replayは未実装である。self-checkで同一CSV同士の全誤差0を確認済みだが、
これは比較配管の検査であり、実機validationではない。
