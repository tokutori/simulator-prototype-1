# Roadmap

IDはrepository内で一意とし、完了条件が満たされるまで`done`にしない。

| ID | Status | Work | Completion criteria |
| --- | --- | --- | --- |
| SIM-001 | done | no_std rigid 6DoF core | unit test、RK4、quaternion、FRD/NED contract |
| SIM-002 | done | JSON aircraft contract v0.10 | strict parse、model validation、schema、sample status、table外policy、environment/controller/failsafe/degraded-air-data/command-conditioning/独立sensor-clock profile |
| SIM-003 | done | servo/sensor/native closed loop | rate/lag/limit test、CSV smoke run |
| SIM-004 | done | steady-glide trim、4-state longitudinal linearization、host固有値解析 | trim residual test、QX-18 mode regression、PNG/CSV report |
| SIM-005 | implemented | JSBSim independent comparison | 同一dataset/inputの5秒replay差をreport |
| SIM-006 | implemented | gust and ground effect | 誘導抗力簡易式、1−cos pulse、unit test、environment sweep。実機validity rangeは未同定 |
| SIM-007 | partial | sensor timing and faults | 独立clock、status/CRC、4-device fault、I²C NACK、固定failsafe反証、SDP degraded mode、critical read 4連続後の気圧基準保持reinitializationを実装。未同定transport delay/noise、SDA/SCL bus固着とphysical recoveryが残る |
| SIM-012 | done | public QX-18/electronics sample | 固定source、仮定、valid range、datasheet profileをJSONと文書で追跡 |
| SIM-013 | implemented | deterministic model-uncertainty stress cases | 範囲の由来を明示し、全caseをCSV/PNG化。実機同定後に範囲を更新 |
| SIM-014 | implemented | aerodynamic-envelope boundary test | strict停止、legacy endpoint保持、BR訓練用再構成をCSV/PNG比較 |
| SIM-015 | implemented | launch scheduling sweep | 135 caseでpull-out scheduleを比較し、再浮上0と高迎角trade-offを確認 |
| SIM-016 | done | shared no_std FBW core | hostとRP2040が同じf32制御state machineを実行し、thumbv6m buildと回帰testに成功 |
| SIM-017 | implemented | no-reascent control stress | launch/glide damping phase分離、100-case gust sweep、actual-UF2 gust比較。実機制御則としては未validation |
| SIM-008 | implemented | RP2040 virtual platform | actual UF2がdatasheet-level virtual I/Oでloopを閉じる。実時間/電気validationは対象外 |
| SIM-009 | partial | RC log validation | 共通CSV比較/残差toolは実装。logger adapter、doublet log、parameter fit、holdout実測が残る |
| SIM-010 | planned | HPA TF identification | target機の係数、delay、不確かさを更新 |
| SIM-011 | planned | aeroelastic extension gate | modal testとcontroller bandwidthから要否判定 |
| SIM-018 | implemented | interactive first/third-person viewer | Three.js replay、actual-UF2 live bridge、keyboard/gamepad button/axis、manual/shared/auto log、TEA state。browser visual QAと実cockpit操作試験は継続 |
| SIM-019 | implemented | production-UF2 pilot input/rudder path | ADC 3ch、button GPIO、I2C sensor、dual PWMをactual firmware/rp2040jsで閉loop化。実input hardware選定、電気fault、physical HILは継続 |
| SIM-020 | done | rp2040js real-time monitor | processing ms、wall-clock ratio、lag、deadline missをtelemetry化し、性能不足をTEA warning stateで常時表示 |
| SIM-021 | implemented | optional WebXR cockpit | Three.js公式VRButton、seated local reference space、head trackingを保持するaircraft rig、XR TEA state、非対応表示。実headset visual/comfort試験、XR HUD/controllerは継続 |

`SIM-008`は`embedded-rust-playground`のUF2 load、`VTOR=0x10000100`、vector-tableのSP/PC、
virtual I2C/PWM boundaryを参照した。BNO055/AS5600/SDP810/DPS310互換registerをactual
firmwareが読み、50 Hz PWMをRust plantへ戻す20秒runを実行済み。10～500倍の加速率sweepから
50倍をdefaultとしたが、非cycle-accurate timing、analog/electrical fault非対応を制約として引き継ぐ。
