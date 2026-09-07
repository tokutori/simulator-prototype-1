# 飛行記録と飛行後analysis

## 記録終了条件

`sim-cli`の標準記録horizonは120秒です。nominal 10 m/sなら約1.2 km分を収容します。
これは飛距離の目標値や強制値ではありません。次のいずれかで先に終了します。

- 水面への到達: `termination=surface-contact`、最終CSV行の`surface_contact=true`
- modelで許可された空力table範囲からの逸脱: `termination=aero-envelope-exit`
- 120秒への到達: `termination=duration`

したがって、標準sampleが約236 mで終了するのは200 mの記録制限ではなく、現在の機体modelと
controllerがその位置で着水するためです。飛行をしていない残り区間を1 kmまで補間しません。
`--duration <SECONDS>`で明示的にhorizonを変更できます。

## Web viewer

viewerは`surface_contact`を0/1またはfalse/trueとして読み取ります。着水した記録は
`WATER CONTACT`、着水flagなしで末尾へ達した記録は`END OF RECORDING`と表示します。

`Flight analysis`または終了bannerの`Review flight`を押すと、現在の記録を別tabで開き、
次を表示します。

- start中心50 m ring付き平面軌跡、range、実移動track
- 高度、対気速度、姿勢、飛行経路角
- pilot elevator/rudder入力
- manual/automatic/mixed commandと実elevator/rudder舵角

analysisには全frameを保存し、統計もグラフも間引く前の記録を用います。等間隔の間引きによる
短時間の振動・最大値の欠落を避けます。120秒/100 Hzの12,001 frameを保持する回帰試験があります。
データは同一originのIndexedDBへ記録ごとのUUIDで保存するため、複数のanalysis tabが互いの
記録を上書きしません。ブラウザの保存容量やpopup制限で失敗した場合は画面へ表示します。
保存は永続バックアップの代わりではありません。必要な記録はCSVもダウンロードしてください。

actual-UF2記録にはfirmware sequence/time、automatic_valid、安全処理後指令、実際に受信した
PWM指令とその受信時刻を保持します。automatic_valid=falseの区間は自動指令のグラフを切り、
ゼロ指令が計算されたと誤表示しません。古いCSVでfirmware証拠がないものはunavailableとして
区別します。表示補間はfirmware sequence/timeを補間しません。

計測境界は異なります。PWM receivedは出力ピン上の有効なパルスから得た指令であり、
Mixed commandはfirmwareのソフトウェア指令、Actualはサーボ動特性を経た模擬舵角です。
これらの差は即座に制御不良を意味しません。CSV内の時刻と更新周期を合わせて比較してください。

`release_mcu_time_us`はpreflight終了時のMCU時計で、機体時刻0秒への対応点です。
MCU/PWM絶対時刻からこれを引いて秒へ換算すると機体時計と比較できます。
`plant_interval_start_s`は、その行に記録した指令を与えた積分区間の開始時刻で、
`time_s`は区間末尾の機体状態時刻です。両者を同時刻の応答とみなさないでください。
