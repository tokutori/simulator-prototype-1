# Architecture and contracts

## Dependency direction

```text
fbw-control-core <------ sim-cli host controller
fbw-input-core ----------> fbw-rp2040 UF2
       ^
       |       fbw-safety-core
       |              ^
fbw-rp2040 UF2 --------+-> rp2040js -> plant-bridge -> flight-dynamics-core
```

`flight-dynamics-core`は`std`、serde、filesystem、clock、random、UIに依存しない。
integration step、wind、control、modelを毎回引数で受け取り、次stateを返す。

PNG描画には[Plotters 0.3.7](https://docs.rs/plotters/0.3.7/plotters/)の
`BitMapBackend`をhost CLIだけで使用する。描画失敗を見落とさないよう、backendを
dropするだけでなく`present()`の`Result`を必ず処理する。

`fbw-control-core`も`no_std`で、RP2040向けに`f32`を使う。host adapterはmodel JSONと
virtual `SensorSample`を同じinput structへ変換する。virtual platformはactual UF2の
I²C/PWMだけを介して制御し、TypeScript版FDMやfirmware専用mock registerを持たない。
`fbw-safety-core`も`no_std`で、sensor validityからhold-last/fixed-failsafe/rearmを決定する。
failsafe舵角はaircraft model contractに置き、QX-18訓練profileでは正を機首下げとして
`+0.75 deg`を使う。これはmodel依存の暫定値であり、core自体へ機体固有値を埋め込まない。
adapterはsensor別のcriticalityを決める。SDP810単独faultでは最後のvalid airspeedを保持し、
raw-invalid診断を立てたまま他のsensorで制御を継続する。BNO055/AS5600/DPS310 faultは
control-criticalとしてsafety gateへ`None`を渡す。

firmwareのsensor lifecycleは`Active`、`Settling`、`RetryAfter`のenumで表し、未初期化なのに
read可能、またはsettling中なのに正常sampleを返す組合せを作らない。`RetryAfter`は離陸時の
pressure referenceだけを保持し、driver stateを作り直しても相対気圧高度の原点を変えない。

Three.jsはhost visualization adapterであり、dependency directionへphysicsの逆流を
作らない。

```text
sim-cli CSV -----------------------> replay parser ----> Three.js cameras/HUD
browser pilot input -> WebSocket -> virtual ADC/GPIO -> rp2040js(actual UF2)
                                                    -> PWM0A/PWM0B -> servo -> Rust FDM
```

browser入力は正規化pilot demandとanalog/buttonの種別だけを送る。manual/automaticのauthority
blend、sensor処理、control、PWM生成はactual RP2040 UF2で行う。servo dynamicsと6DoF stepは
Rust plant側に置く。描画frame補間とchase-camera smoothingは
visual-onlyであり、plant stateやcontroller telemetryへfeedbackしない。

Web appの実行状態は[The Elm Architecture](https://guide.elm-lang.org/architecture/)に従う判別可能unionの`AppState`をsingle source of
truthとする。`update(AppState, AppMsg) -> (AppState, Effect)`は純粋で、WebSocket接続などを
`Effect`として外へ返す。Three.js/DOMは`present(AppState)`のadapterであり、
`mcu-running`なのにactual-UF2 backend情報がない状態を構築しない。

```text
Rust plant observation
  -> BNO055/AS5600/SDP810/DPS310 register encoding
  -> RP2040 I2C MMIO
  -> shared controller in actual UF2
  -> shared sensor-fault safety gate
  -> 50 Hz servo PWM
  -> command decode
  -> Rust actuator model
  -> Rust nonlinear FDM
```

RP2040 ROM intrinsicは`rp2040js`のdirect-vector entryでは使えないため、HALの
`disable-intrinsics` featureでportable compiler intrinsicsを使う。これは実機でも動く
build選択であり、simulation時だけ別firmwareを生成するfeatureではない。

## Coordinate and sign contract

- body: `x` forward、`y` right、`z` down
- navigation: NED (`x` north、`y` east、`z` down)
- attitude quaternion: body vectorをNEDへ回転
- angular rate: body axesのright-hand positive `p/q/r`
- position altitude: `altitude = -position_ned.z`
- aerodynamic angle: `alpha = atan2(w_air, u_air)`
- sideslip: `beta = asin(v_air / V)`
- unit: SI。JSONのhuman-facing angleだけfield名に`_deg`を明示し、coreではrad

Inertiaは次のliteral tensor elementである。

```text
[ Ixx   0  Ixz ]
[   0 Iyy    0 ]
[ Ixz   0  Izz ]
```

航空分野で使われる`-Ixz`表記を暗黙に採用しない。設計班dataの変換時に符号を明示する。

## Aerodynamic contract

longitudinal base coefficientは`alpha` tableから線形補間する。範囲外policyはmodel contract
v0.11.0の`terminate`または`clamp-and-flag`で明示する。`terminate`は次stepへ進まず
`aero-envelope-exit`で終了する。`clamp-and-flag`はsoftware test専用であり、endpoint保持に
よって範囲外modelが妥当になるわけではない。どちらもvalidityをCSVとsummaryへ返す。

```text
CL = CL_table(alpha) + CL_delta_e delta_e
Cm = Cm_table(alpha) + Cm_q q c/(2V) + Cm_delta_e delta_e
CY = CY_beta beta + CY_p p b/(2V) + CY_r r b/(2V) + CY_delta_r delta_r
Cl = Cl_beta beta + Cl_p p b/(2V) + Cl_r r b/(2V) + Cl_delta_r delta_r
Cn = Cn_beta beta + Cn_p p b/(2V) + Cn_r r b/(2V) + Cn_delta_r delta_r
```

force係数のbasisを`force_coefficient_basis`で必須指定する。
`wind-axes`ではlift/drag/sideを直交wind frameからalpha/betaの両方でbodyへ回転する。
`stability-lift-drag-body-side`ではCL/CDは縦のstability平面、CYはbodyの全横力であり、
BRの公開式と同じ小横滑り近似を明示的に維持する。body CYに抗力横成分を追加したり、
wind-side係数として再回転したりしない。QX-18の二つの再構成は後者、illustrativeは前者。
両者を同じ係数値のまま交換してはいけない。BR近似は大横滑りの実機validationを意味しない。
参照: [作者の座標系・横力の定義](https://mtkbirdman.com/unity-aerodynamiccalculator)。
momentはreference `S/b/c`でscaleする。
JSONはmoment reference pointを必須metadataとして持つ。

## Release velocity contract

`initial_state.velocity`はtagged unionで、`frame=ground-relative`なら`speed_mps`、
`flight_path_deg`（上向き正）、`track_deg`（北から東向き正）を指定する。
機体姿勢とは独立したNED対地速度を構築し、風はその後の対気量にのみ影響する。
training sampleは人による補助発進の対地5 m/s、飛行経路−3°、北向きを保持する。
`frame=air-relative`なら`airspeed_mps`と`alpha_deg`を指定し、初期beta=0とする。
局所風（開始地点のgustを含む）を加えて対地速度を構築する。
旧v0.10.0の曖昧な`initial_state.airspeed_mps`形式は受理しない。
nativeと外部controller経路は同一の初期状態生成関数を使う。

ground effectはmodelごとに無効化できる。現在は
`CD = CD_base + (CGE - 1) k_induced CL^2`だけを適用し、lift、downwash、momentは変えない。
1−cos gustはnorth位置を独立変数とする有限長wind pulseで、coreへは各stepの明示的
`Environment`として渡す。いずれもhidden clockやglobal environment stateを持たない。

## Determinism

- fixed time step
- hidden global stateなし
- sensor random noiseなし。IMU/差圧/静圧/迎角の独立sample clock、量子化、bias、pressure responseは明示state
- servo stateはcallerが所有
- f64 arithmeticとtable順序を固定

異なるCPU/compiler間のbitwise一致は現時点のcontractにしない。同一binary・同一入力の
repeatabilityを最初のcontractとし、cross-platform tolerance testを追加予定とする。

## Safety boundary

default sampleの`validation_status`は`training-only-unvalidated-high-alpha-model`である。
CLIはstatusをrun summaryへ必ず表示する。JSON自身がsource、適用先、authority、assumptionを
持つ。strict QX-18 modelと訓練用BR挙動再構成を別fileにし、flight-test同定結果と混同しない。
