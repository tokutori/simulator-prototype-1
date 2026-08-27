# RP2040 virtual platform boundary

これはactual target binaryを通すsoftware-in-the-loop試作であり、cycle-accurate HILではない。
`embedded-rust-playground`と同じくUF2をXIP flashへloadし、`VTOR=0x10000100`、vector tableの
initial SP/reset PCから実行する。firmware sourceにはsimulator専用peripheralや`cfg(sim)`を
置かない。

## Closed-loop path

| boundary | implementation | validated here | not validated here |
| --- | --- | --- | --- |
| controller | shared `no_std` f32 core | state machine、量子化後input、actual target codegen | control lawの実機安全性 |
| IMU | BNO055 0x28 | CHIP_ID=0xA0、NDOF、SYS_STATUS=5/SYS_ERR=0、gyro Y/Euler pitch | fusion accuracy、axis remap、磁気外乱、振動 |
| AoA | AS5600 0x36 | STATUSのMD/MH/ML、RAW ANGLE 12 bit、wrap | vane空力、取付zero、linkage、flutter |
| airspeed | SDP810 0x25 | 0x3615、9-byte frame全3 wordのCRC-8、nonzero scale | pitot係数、tube lag、水滴、position error |
| barometer | DPS310 0x77 | SENSOR/COEF ready、PRS_RDY 32 Hz生成/read-clear/20-read timeout、coefficient decode、polynomial compensation | package stress、temperature drift、port dynamics |
| actuator | KRS-compatible PWM | 20 ms、1000～2000 us、RP2040 PWM MMIO | loaded travel、backlash、current、brownout |
| plant | Rust nonlinear 6DoF | same plant as native reference run | real aircraft coefficient validity |

## Pilot input and output pin contract

production firmwareはpilot hardwareとsimulatorで同じpin contractを使う。ADCはRP2040の12-bit
countとして読み、中央±164 countをdead zoneにする。buttonはactive-lowで、押されている間は
対応axisをfull demandへoverrideし、反対button同時押しはneutralにする。

| GPIO | function |
| ---: | --- |
| 26 / ADC0 | elevator analog axis、正は機首下げ |
| 27 / ADC1 | rudder analog axis、正はright-yaw要求 |
| 28 / ADC2 | automatic authority、0=manual、4095=auto |
| 10 / 11 | elevator nose-up / nose-down button |
| 12 / 13 | rudder left / right button |
| 16 / PWM0A | elevator servo PWM |
| 17 / PWM0B | rudder servo PWM |
| 18 / 19 / 20 / 21 | invalid / control tick / deadline miss / safety diagnostic |

manual/automatic commandはauthorityで線形blendする。両舵とも1000～2000 usへ制限する。
analog joystick、button、authority potentiometerの電気故障、接点bounce、断線検出は未実装であり、
実配線決定後にpull-up、range plausibility、redundancyをrisk assessmentする必要がある。

BNO055の650 ms POR、SDP810の最初の8 ms、DPS310のmeasurement設定をfirmware側で扱う。
BNO055公式datasheetはfusionが主に人の動作用で、持続する大加速度を重力と誤認し得るため、
status正常だけを姿勢精度の証拠にはしない。
KRS-4034HV公式manualのPWM許容はcycle 3～30 ms、normal pulse 700～2300 usなので、出力は
電気的protocol範囲内である。ただし1000～2000 usを舵面±10 degへ対応させるlinkage比は
試作仮定であり、実機horn geometryと実舵角sensorで校正する。

## Timing limitation

`rp2040js`はRP2040 boot ROMのintrinsic tableを提供しないため、firmwareは
`rp2040-hal/disable-intrinsics`でportable compiler routinesを使用する。実機互換buildで
あり、simulation専用logicではない。firmwareは1 MHz timerで処理時間を差し引いて10 ms周期を
作り、超過時はGPIO20をhighにする。加速率10/20/50/100/200/500を3秒runで比較すると、
10～50倍は302 update・deadline miss 0だったが、100/200/500倍は241/121/51 updateへ崩れた。
defaultを50倍へ下げた。ただしこれはcycle accuracyの証拠ではなく、summaryは常に
`timing_validated=false`を返す。

物理MCUをPCへ接続するHILでは、同じplant bridgeの外側adapterだけを交換し、少なくとも
deadline、interrupt jitter、I²C timeout、watchdog、servo電源、level shifting、brownout、
disconnect時の固定failsafe舵、sensor再初期化、watchdogを追加試験する。

## Result

20秒nominal runは再浮上0、正飛行経路角sample 0、最大飛行経路角-1.23 degだった。native
hostとのaltitude RMSE 0.0290 m、flight-path RMSE 0.0513 deg、actual elevator RMSE
0.267 degである。
これはbus/firmware integrationの整合であり、QX-18実機の軌道を約3 cmで当てるという意味ではない。

比較plot: `reports/virtual-platform-comparison.png`。

同じactual UF2へ0.5/1.0/2.0 m/sの決定的上昇gustを与えると、再浮上は
0/0.029/0.186 m、最大飛行経路角は-0.43/0.50/2.10 degだった。gust区間の最大実舵角は
3.11/5.46/9.77 degで、2 m/s caseの余裕は約0.23 degである。ただし舵角総変動は増えており、
再浮上防止を保証できる結果ではない。
比較plotは`reports/virtual-platform-gust.png`。

## Sensor validity and failsafe experiment

共有`fbw-safety-core`は起動時10連続validでarmし、1～2 invalid updateはlast commandを保持、
3回目でmodel contract指定の固定failsafe舵へ移行する。QX-18訓練profileは正を機首下げとして
`+0.75 deg`を使う。復帰には20連続validを要求し、controller stateもclean startする。
ただしこれはpitch/AoA/barometric altitudeを作れないcontrol-critical faultに適用する。
SDP810だけのfaultでは最後のvalid airspeedを保持し、残るIMU/AoA/barometerで制御を継続する。
GPIO17はarming/failsafe、GPIO18はdegradedを含むraw invalid、GPIO19はcontrol updateを示す。

SDP CRC/NACKを1または3 update壊すとGPIO18は異常を示すがfailsafeは0で、再浮上も0だった。
BNO status、AS5600 magnet、DPS readyを3 update壊すとfailsafe 1回、`invalid→failsafe`は
2 update差（3回目）、`valid復帰→rearm`は19 update差（20回目）。7秒高度差はSDP CRC 0、
SDP NACK +0.25 cm、control-critical 3種は+1.16 cmだった。比較plotは`reports/fault-injection.png`。

永続sensor lossについて、hold-last、固定0～3 degを6つの開始時刻で比較した。0 degは最大
5.607 m、hold-lastは3.374 m再浮上し、固定0.50 degにも0.042 m残った。固定0.75/1.0/1.5 degは
nominal全開始時刻で再浮上0だったため0.75 degを暫定値にした。ただし17個のmodel stressを
2秒故障へ掛けると、0.75/1.0/1.5 degは7/7/10 caseで再浮上し、最悪0.920/0.679/0.251 mだった。
固定舵だけでrobustな解は得られていない。actual UF2の永続BNO faultはnominal 6時刻で再浮上0だが、
0.5/1.0秒faultは接水し、2.0秒caseの最大経路角は-0.055 degしか余裕がない。
比較plotは`reports/failsafe-command-sweep.png`、actual-UF2結果は
`reports/failsafe-bno-start-*.json`、時系列は`reports/persistent-fault-response.png`に保存する。

永続SDP NACKはdegraded modeで別評価した。0.25 sの経路先読みと、release高度から
0.75～2.0 mを失う間の相対気圧高度blendを追加した結果、0.5/1/2/3/5/7秒faultの全てで
再浮上0・接水0となった。0.5秒caseは最大経路角-1.37 deg、正経路角0、command/actual舵角の
総変動93.4/60.0 degである。時系列は`reports/degraded-airspeed-timing.png`。
最も早い0.5秒faultへ17個のmodel stressを掛けても全caseで再浮上0・接水0、failsafe 0、
deadline miss 0だった。adverse cornerは最大経路角-1.04 deg、高度0.278 mで12秒を終えた。
ただし高度閾値は再構成軌道から選んだ仮定で、実機同定分布ではない。
plotは`reports/degraded-airspeed-early-robustness.png`。

DPS310公式datasheetの`MEAS_CFG.PRS_RDY`は新pressure resultを示し、pressure register readで
clearされる。virtual deviceは32 Hzでbitを生成し、pressure 3 byte readでclearする。firmwareは
bitが0の100 Hz周期で直前pressureを最大20 read保持し、それを超えると異常にする。controllerも
32 Hzのbarometric値が変化した時だけ鉛直速度を更新し、同一sampleの100 Hz反復をゼロ速度として
混ぜない。40 updateの`dps-stale`固定試験は開始0.23 s後にfailsafeへ入り、20 valid updateで復帰した。

## Datasheets

- [Bosch BNO055 rev.1.8](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)
- [ams AS5600 v1.06](https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf)
- [Sensirion SDP8xx v1.1](https://sensirion.com/en/media/documents/90500156/6167E43B/Sensirion_Differential_Pressure_Datasheet_SDP8xx_Digital.pdf)
- [Infineon DPS310 v1.2](https://www.infineon.com/assets/row/public/documents/24/49/infineon-dps310-datasheet-en.pdf)
- [Kondo KRS series manual](https://kondo-robot.com/w/wp-content/uploads/KRS-series_manual_Download-En.pdf)
