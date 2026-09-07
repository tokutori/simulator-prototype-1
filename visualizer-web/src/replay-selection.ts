/** Monotonic intent generation: slow file/network completion cannot replace newer selection. */
export class ReplaySelection {
  private generation = 0;
  current(): number { return this.generation; }
  select(): number { return ++this.generation; }
  accepts(generation: number): boolean { return generation === this.generation; }
}
