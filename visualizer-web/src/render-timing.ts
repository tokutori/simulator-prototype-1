export type RenderStatus = { tag: 'measuring' } | { tag: 'measured'; fps: number; slow: boolean };
export interface RenderTiming { startMs: number; frames: number; status: RenderStatus }
export type RenderMsg = { type: 'reset'; nowMs: number } | { type: 'frame'; nowMs: number };

export function updateRenderTiming(state: RenderTiming, msg: RenderMsg): RenderTiming {
  if (msg.type === 'reset' || msg.nowMs < state.startMs) {
    return { startMs: msg.nowMs, frames: 0, status: { tag: 'measuring' } };
  }
  const frames = state.frames + 1;
  const elapsed = msg.nowMs - state.startMs;
  if (elapsed < 1000) return { ...state, frames };
  const fps = frames * 1000 / elapsed;
  return { startMs: msg.nowMs, frames: 0, status: { tag: 'measured', fps, slow: fps < 30 } };
}

export function presentRenderTiming(status: RenderStatus): string {
  return status.tag === 'measuring' ? 'DRAW: measuring' :
    `DRAW: ${status.fps.toFixed(0)} fps${status.slow ? ' — LOW FPS / 操作・表示が遅延する可能性' : ''}`;
}

/** Presentation time is wall time, not a capped physics integration step. */
export function presentationDeltaS(previousMs: number, nowMs: number): number {
  return Math.max(0, (nowMs - previousMs) / 1000);
}
