# Prototype completion audit

監査日: 2026-08-31。公開情報の調査基準日は2026-08-22。

## 結論

本repositoryは、鳥人間滑空機用FBWの**software/firmware integration prototype**としては、
nonlinear 6DoF plantからvirtual sensor、production RP2040 UF2、virtual PWM/servoを経てplantへ
戻るloopを構成できている。Interactive viewerもこのactual-UF2 loopをdefaultとして使い、manual、
shared、automatic controlを同じ経路で比較できる。

一方、これは対象実機の飛距離・安全性を予測または保証できるvalidated simulatorではない。
QX-18 training modelの高迎角係数、安定微係数、慣性、負荷時servo response、sensor installation error、
構造柔軟性は実機同定済みではない。現行結果はarchitecture、数式実装、binary integrationを検査する
ためのものであり、実機validationとの境界を維持する。

## User requirement audit

| Requirement | Status | Evidence and boundary |
| --- | --- | --- |
| 公開simulator・論文・実装・datasheetの調査 | complete for stated public cutoff | 隣接`simulator-search`の`research.md`、`sources.md`、`evidence.csv`、機体・電装・launch/control資料に分離 |
| 独立Git repository | complete | `simulator1/.git`で管理し、親`tokutori/`をrepositoryにしていない |
| pure/no_std-friendly FDM | implemented | `flight-dynamics-core`はI/O・allocationなし、固定step RK4、quaternion、FRD/NED nonlinear 6DoF |
| 設計班aero dataとのcontract | implemented | schema v0.10、reference geometry/point、係数table、valid range、環境・controller・独立sensor clockをmachine-readable化 |
| 公開機体sample | implemented with limitations | 公開QX-18/BR情報をstrict modelとtraining envelopeに分離。高迎角側は実測値ではない |
| sensor/servo datasheet model | implemented subset | BNO055、AS5600、SDP810、DPS310、KRS-4034HVの独立sample clock、register/CRC/ready/量子化/PWM/rateを実装。電気noise、配線、負荷電流等は未実装 |
| actual production RP2040 firmware | complete for virtual platform | production UF2をactual rp2040jsへload。host controller fallbackを拒否し、virtual ADC/GPIO/I2Cとdual PWMを通す |
| manual/shared/automatic control | implemented | keyboard/gamepad button/axis、0～100% authority、pilot/automatic/mixed/actual surfaceを別々に記録 |
| TEA and impossible-state modeling | implemented | replay/connecting/running/too-slow/ended/failedとXR availabilityを判別可能union、pure update/presentationで表現 |
| real-time capacity warning | implemented and observed | processing time、real-time ratio、lag、deadline missを表示。性能不足時もfallbackせず常時warning |
| first/third-person visualizer | implemented | Three.jsは描画adapterのみ。Cockpit/Chase、波、25 m ring、50 m label、追従shadow、読みやすいHUD/舵面表示 |
| FHD～4K responsive UI | implemented, device QA pending | shortest-side基準scaleにunit testあり。実display視認距離とcockpit照明下試験は未実施 |
| post-flight analysis | implemented | 着水とrecord終端を区別し、別tabで軌跡、pilot/auto/mixed/actual舵、state時系列を表示 |
| 1 km級record capacity | complete | default horizon 120 s。nominal 10 m/sで約1.2 km相当を収容するunit testあり |
| optional VR | implemented, headset QA pending | Three.js公式WebXR、seated `local` reference space、head-tracked cockpit rig。XR HUD/controllerと実headset comfort試験は未実施 |
| RC/実機validation | workflow only | 共通CSV・残差tool・maneuver/holdout手順まで。logger adapter、parameter fit、RC/HPA実測logは未投入 |
| aeroelastic extension | gate only | wing/tail-boom modal testとcontroller bandwidthの比較前なのでmodelへ未追加 |

## Current deterministic evidence

2026-08-31に`qx18-br-training-envelope.json`を5 m/s、水平から下向き3 deg、10 ms stepで再実行した。

| Result | Host shared controller | Production UF2 on rp2040js |
| --- | ---: | ---: |
| water contact | 23.17 s | 23.24 s |
| final north range | 232.50 m | 233.07 m |
| maximum flight-path angle | -1.24 deg | -1.19 deg |
| positive flight-path samples | 0 | 0 |
| maximum re-ascent | 0 m | 0 m |
| maximum angle of attack | 16.69 deg | — |
| observed firmware control updates | — | 2321 |
| firmware deadline misses in accelerated batch | — | 0 |

Hostとactual-UF2 runの比較はaltitude RMSE 0.0166 m、flight-path RMSE 0.0478 deg、actual elevator
RMSE 0.2160 degだった。これはsoftware path equivalenceであってreal-aircraft accuracyではない。

現行制御は再浮上を生じないが、公開される約2 m降下後の定常滑空像よりpull-out高度損失が大きい。
最大迎角もtraining table上限20 degに近い。実測logなしにgainや係数を調整して軌跡だけ合わせることは
しない。優先入力は発進直後のairspeed/alpha/pitch/q/実舵角、質量・慣性・CG、負荷時servo response、
wing/tail-boom modeである。

Interactive actual-UF2 smokeでは80 telemetryを受けた。このPCで最大平均処理時間4.50 ms/update、
最小real-time ratio 0.74、最大lag 268.9 ms、non-real-time 34 sampleを観測した。この結果は性能不足を
画面に明示すべきcaseであり、rp2040jsのcycle timing validationには用いない。

## Verification rerun

2026-08-31に次を成功させた。

- `cargo fmt --all --check`
- `cargo test --workspace`: 48 unit/scenario test + 1 doc test
- `cargo clippy --workspace --all-targets -- -D warnings`
- production firmware `thumbv6m-none-eabi --release` build
- AJV Draft 2020-12でmodel contract 3件をschema validation
- `virtual-platform`: TypeScript check、5 tests、high-severity audit 0
- `visualizer-web`: TypeScript check、29 tests、production build、high-severity audit 0
- actual-UF2 interactive HTTP/WebSocket smoke: 80 observations
- actual-UF2 23.24 s batch flight and host comparison

Vite production buildにはsingle JavaScript chunkが500 kBを超えるperformance warningが残る。
機能・正しさのfailureではないが、network配布を行う段階ではanalysis pageやThree.jsのcode splittingを
検討する。

## Remaining validation gates

次の作業は追加情報またはphysical hardwareを必要とし、prototype完成と区別する。

1. 実cockpit姿勢でstickと大形buttonを比較し、手袋、振動、水しぶき、拘束、誤押下を評価する。
2. physical RP2040、実sensor/bus exerciser、servo電源・負荷を使うHILでdeadline、jitter、brownout、
   bus固着、再初期化を確認する。
3. RC機でlogger、時刻同期、実舵角、doublet、parameter fit、holdout replayを完了する。
4. 鳥人間実機TF logで係数・遅延・不確かさを更新する。
5. 構造modal testからaeroelastic state追加の要否を判定する。
6. 対応VR headsetでFOV、eye point、frame rate、酔い、入力中の安全性を評価する。

これらを完了するまで、training-only表示と「simulation単独では実機safetyを保証しない」という制約を
外してはならない。
