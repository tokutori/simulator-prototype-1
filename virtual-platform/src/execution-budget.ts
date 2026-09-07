/** Watchdog applies to one bounded unit of work, never the accumulated flight. */
export function advanceUntil(
  done: () => boolean,
  execute: () => void,
  instructionBudget: number,
): number {
  if (!Number.isSafeInteger(instructionBudget) || instructionBudget <= 0) throw new Error('invalid instruction budget');
  let instructions = 0;
  while (!done()) {
    if (instructions >= instructionBudget) throw new Error('virtual MCU made insufficient progress within this step');
    execute();
    instructions++;
  }
  return instructions;
}
