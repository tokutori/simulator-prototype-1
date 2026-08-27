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

長い記録は先頭と末尾を保持して最大1600 frameへ等間隔downsampleしてから、同一originの
`localStorage`を介してanalysis tabへ渡します。元のCSVやinteractive download logは変更しません。
