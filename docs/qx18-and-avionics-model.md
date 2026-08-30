# QX-18と電装modelの根拠

確認日は2026-08-25。model JSON自体にも固定URLと適用範囲を保持する。

## 機体

sampleは九州大学鳥人間チームの2018年度滑空機QX-18を基礎にする。チームの
[公式三面図](https://www.q-birdman.jp/history/images/qx-18.pdf)には、総重量97.0 kg、
翼幅25.2 m、主翼面積18.0 m²、MAC 0.757 m、巡航速度10.0 m/s、水平尾翼面積
1.37 m²、水平尾翼容積比0.424、舵角±10 degが記載されている。

実行modelの93.875 kg、18.042 m²、25.133 m、9.7 m/s、慣性、trimおよび空力微係数は、
[BR Simulatorの固定revision](https://github.com/mtkbirdman/mtkbirdman.com/blob/736fa36bcb3bfd65c7808b19e9d215682af2671a/BirdmanRallySim/Plane/AerodynamicCalculator.cs)
から再構成した。公式値との差は丸めだけとは限らないため、値を混ぜずBR Simulator側を
一組のdatasetとして採用した。

BR SimulatorのUnity座標をFRDへ変換した慣性tensorは次である。

```text
Ixx = 876.5039836 kg m2
Iyy =  76.0000000 kg m2
Izz = 946.4960164 kg m2
Ixz =  -5.9606070 kg m2  (literal matrix element)
```

`qx18-public-reconstruction.json`のlongitudinal tableは同sourceの通常飛行式をground
effectなし・無操舵で評価したもの。sourceの`CLMAX` clippingを失速modelとは見なさず、
tableを-5～8 degに制限し、範囲外へ出た時点でsimulationを停止する。

発進制御のsoftware stress test用には、別の`qx18-br-training-envelope.json`を用いる。
これは固定revisionのBR式を-12～20 degで評価し、主翼・尾翼それぞれの`CLMAX = 1.7`
clipping、alphaの3乗で増えるparasite drag、induced dragを再現する。実測失速dataではなく、
lift drop、hysteresis、dynamic stall、剥離、非定常空力を持たないため、訓練挙動再現に限る。

default scenarioは利用者から示された補助発進条件に従い、platform edgeで5.0 m/s、
水平から下向き3 deg、高度10.5 mから開始する。初期迎角1.682 degを保つため、初期pitchは
`-3 + 1.682 = -1.318 deg`とした。走行中の人力とplatform contactはmodel化していない。
低速中は参考controllerが仮想AoA channelで6 degを目標にして加速降下し、servoと機体の
応答遅れを見越して6.0～7.5 m/sで
浅い負の飛行経路角へ滑らかにblendする。固定pitch追従は、diveで得た運動energyを
再浮上へ戻したため廃止した。現在は`pitch - alpha`とpitch-rateによる0.10 s先読みで
引き起こしを減衰させ、正の飛行経路角にはdown側のenvelope protectionを加える。
SDP810が無効なら先読みを0.25 sへ伸ばし、release時の気圧高度から0.75～2.0 m失う間にも
pull-outをblendする。これらの高度値は再構成nominal軌道から選んだもので、実flight同定値ではない。
発進中のpitch-rate dampingを強くすると引き起こしが深くなるため、5 m/s発進時は0.05 s、
7.5 m/s以上かつ飛行経路角が-3 degまで回復した時点でglide phaseをlatchし、0.25 sかけて
0.60 sへ移行する。これは100-caseの決定的gust sweepによるsoftware-test設定であり、
実機同定済みgainではない。
tracking commandには15 msの一次整形を入れ、envelope protectionとの合成後を`±10 deg`、
`352.94 deg/s`へ制限する。保護項はtracking low-passを迂回する。これはPWM要求が実servo modelより
速く反転していた問題へのsoftware-test上の対処であり、実機同定済みcommand shaperではない。
また32 Hzのbarometric高度は、100 Hz loopで同じsampleをゼロ速度として繰り返し混ぜず、値が
変化した時だけ経過時間で差分する。0.25 s filterと`-0.25 m/s`の対地barrierを使うが、
pressure noise/transport delay未実装の決定的stress設定である。
発進前servo presetもschema上は明示するが、QX-18 sampleは公開根拠がないため0 degとした。

この戦略は[発進・制御戦略の調査](../../simulator-search/launch-and-control-strategy.md)に基づく。
摂南大学の公開論文では、5～6 m/s発進後に約2 m降下して約8.5 m/sを得てから引き起こす
過程が説明されている。現在のmodelも8.5 m/sには約2.6 mの高度損失で達するが、
引き起こしが浅い-3 degまで戻る時点では約5.01 mを失う。QX-18再構成値の9.7 m/sまで
無推力で加速する位置energyは、dragを無視しても3.52 mを要するため、5 m級の降下自体は
物理的に不可能ではない。しかしreference commandは発進・引き起こし中に±10 degへ達し、
host評価で約0.92 sがlimit付近になる。servo rateにより実舵は連続でも、公開チーム資料が
重視する一回の穏やかな定常移行を実機相当で再現したとはまだ言えない。同じ2 m降下後の滑らかな巡航を
再現できておらず、
実機flight logで同定するまで軌道予測値として扱わない。このAoA channelも
BNO055/SDP810/DPS310だけでは直接得られず、実機で同じcontrol lawを使うなら
風向sensorまたはstate estimatorが必要である。

## 実搭載例と部品datasheet

[鳥科2023電装資料](https://771-8bit.com/blog/birdman-glider-avionics/)で、滑空機への
KRS-4034HV ICS、BNO055、DPS310、SDP810-500Paの採用を確認した。これはQX-18の
搭載品を示す資料ではなく、鳥人間滑空機で成立した構成のreferenceである。

| component | modelへ入れた特性 | 一次資料 | model上の扱い |
| --- | --- | --- | --- |
| Kondo KRS-4034HV ICS | 11.1 Vで0.17 s/60 deg、最大270 deg | [公式製品仕様](https://kondo-robot.com/product/krs-4034hv-ics) | QX-18舵角±10 deg、最大速度352.94 deg/s。lag/deadband/command量子化は仮定 |
| Bosch BNO055 | fusion mode 100 Hz、Euler 1/16 deg、gyro 1/16 deg/s、accel 0.01 m/s² | [datasheet rev.1.8](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf) | 100 Hz sample-and-hold、量子化、明示bias。fusion algorithm内部誤差は未model |
| Sensirion SDP810-500Pa | ±500 Pa、16 bit、scale factor 60 counts/Pa、step response τ63 < 3 ms | [datasheet v1.1](https://sensirion.com/file/datasheet_sdp800-d/) | 1/60 Pa量子化、3 ms一次遅れ、±500 Pa飽和、32 Hz読出し |
| Infineon DPS310 | 1～128 Hz、data resolution 0.06 Pa、standard precision 0.35 Pa RMS | [datasheet v1.2](https://www.infineon.com/dgdl/Infineon-DPS310-DataSheet-v01_02-EN.pdf?fileId=5546d462576f34750157750826c42242) | 鳥科実装に合わせ32 Hz、0.06 Pa量子化、ISA換算高度 |
| ams AS5600 | I²C 0x36、12 bit/360 deg、RAW ANGLE 0x0C～0x0D | [datasheet v1.06](https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf) | AoA vaneの試作用profile。鳥科搭載実績を示すものではなく、取付zero/リンクageは未同定 |

BNO055は実績を再現するためのprofileであり、新規実機の部品選定を推奨する意味ではない。
また、BNO055の融合姿勢をそのままFBWへ使うなら、校正状態、磁気外乱、axis remap、起動時間、
異常register値のtestが別途必要である。

## 現在のvalidation境界

このsampleで確認できるのは、公開値から組んだnominal rigid-body plantに対するcontroller、
sample-and-hold、量子化、pressure response、servo rate/lag/deadbandのsoftware挙動である。
QX-18の実飛行軌道、stall、強いgust、ground effect、構造柔軟性、安全性は予測しない。
5.0 m/s発進caseをstrict modelへ与えると0.10 sで上限8 degを超え、
`aero-envelope-exit`で停止する。従来の8 deg endpoint保持では-3 degへ戻るまで4.52 m、
BR訓練用再構成では5.01 mの高度を失った。両者の差0.48 mは、table範囲外policyが軌道を
変える具体例である。訓練用modelでも実測validationにはならず、post-stall dataまたは
実flight logが必要である。比較は`reports/aero-envelope-comparison.png`へ出力する。

training modelのdefault regressionは`max_flight_path_deg <= 0`、`max_reascent_m == 0`、
`positive_flight_path_samples == 0`を確認する。これは再浮上しないreference controlの
software acceptanceであり、空力table外区間や実機安全性を合格扱いにするものではない。

次に必要なのは次の実測である。

- actuator commandと実舵角のstep response、backlash、負荷時速度、電源電圧低下
- pitot assembly全体の係数とtube response
- IMU mounting orientation、振動下noise、bias、温度drift
- 実機のdoublet/自由応答logによる慣性・安定微係数・構造modeの同定
- measured parameter uncertaintyを振るMonte Carlo test
