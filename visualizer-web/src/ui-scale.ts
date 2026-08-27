const referenceWidth = 1920;
const referenceHeight = 1080;

export function uiScaleForViewport(width: number, height: number): number {
  if (!Number.isFinite(width) || !Number.isFinite(height) || width <= 0 || height <= 0) return 1;
  const shortestSideFit = Math.min(width / referenceWidth, height / referenceHeight);
  return Math.max(1, Math.min(2, shortestSideFit));
}

export function applyUiScale(width: number, height: number): void {
  // Percentage preserves the browser's own base font preference. At 4K this
  // becomes 200%, while a 21:9 FHD-height display remains at 100%.
  document.documentElement.style.fontSize = `${uiScaleForViewport(width, height) * 100}%`;
}
