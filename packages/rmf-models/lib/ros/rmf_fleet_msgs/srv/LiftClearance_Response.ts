/* This is a generated file, do not edit */

export class LiftClearance_Response {
  static readonly FullTypeName = '';

  static readonly DECISION_CLEAR = 1;
  static readonly DECISION_CROWDED = 2;

  decision: number;

  constructor(fields: Partial<LiftClearance_Response> = {}) {
    this.decision = fields.decision || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['decision'] !== 'number') {
      throw new Error('expected "decision" to be "number"');
    }
  }
}

export default LiftClearance_Response;
