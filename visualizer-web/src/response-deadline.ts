/** Wall-clock request liveness, independent of firmware/plant simulated time. */
export class ResponseDeadline {
  private state: { tag: 'idle' } | { tag: 'waiting'; untilMs: number } = { tag: 'idle' };

  constructor(private readonly timeoutMs: number) {
    if (!Number.isFinite(timeoutMs) || timeoutMs <= 0) throw new Error('invalid response timeout');
  }

  begin(nowMs: number): void {
    if (this.state.tag === 'waiting') throw new Error('overlapping plant requests');
    this.state = { tag: 'waiting', untilMs: nowMs + this.timeoutMs };
  }

  complete(): void { this.state = { tag: 'idle' }; }

  expired(nowMs: number): boolean {
    return this.state.tag === 'waiting' && nowMs >= this.state.untilMs;
  }
}
