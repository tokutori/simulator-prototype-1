# Visualizer UI scaling

## 対象と基準

viewerは1920×1080をreference resolution、3840×2160をtarget上限とする。UIのscaleは
viewportの短辺側へfitさせる。

```text
s = clamp(1, min(viewport_width / 1920, viewport_height / 1080), 2)
root font size = browser default × s
```

これはUnreal EngineのDPI Scalingにある`Shortest Side`相当である。FHDを1、QHDを約1.333、
4Kを2とする。3840×1080のような横長画面は高さがFHDのままなので1となり、UIだけが過大に
ならない。FHD未満ではscaleを1未満にせず、既存のresponsive breakpointでreflowする。

## CSS contract

- JavaScriptはroot font sizeを百分率で設定し、browser既定font sizeを上書きしない。
- HUD、control、間隔、文字は`rem`、配置領域は`vw`/percentageを基本とする。
- 1 px borderは情報量ではなく境界線なので固定pixelを許容する。
- clickable controlはFHDで最低1.75 remとし、24 CSS px以上を確保する。
- notch等の領域には`env(safe-area-inset-*)`を使う。
- WebGL canvasはviewport全体を使い、renderer pixel ratioは2以下に制限する。3D scene解像度と
  2D UI scaleを別に扱う。

browser zoomはviewportのCSS pixel数を変えるため、4Kで200% zoomした場合は自動scaleが1へ
戻り、browser zoomとの二重拡大を避ける。文字とcontrolは画像化せず、200%拡大時も内容と
機能を失わない構造を維持する。

## 根拠

- Unreal Engine DPI Scaling: resolution-independent UI、Shortest Sideがmost common setting
- W3C WCAG 2.2 SC 1.4.4: textを200%へ拡大してもcontent/functionを失わない
- W3C WCAG 2.2 SC 1.4.10: 拡大時にreflowし、不要な二方向scrollを避ける
- W3C WCAG 2.2 SC 2.5.8: pointer targetは原則24×24 CSS px以上

## Verification matrix

| Viewport | Expected scale | 主確認 |
| --- | ---: | --- |
| 1920×1080 | 1.000 | baseline HUD、control、flight event |
| 2560×1440 | 1.333 | 比例拡大、重なりなし |
| 3840×2160 | 2.000 | FHDと同じ視角相当のUI |
| 3840×1080 | 1.000 | ultrawideで過大化しない |
| 1280×720 | 1.000 | responsive layoutへ移行 |

自動testはscale計算と上下限を検証する。最終的な視認距離、display物理寸法、cockpit内での
可読性は実displayとVR deviceで確認する。
