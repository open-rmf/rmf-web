/* This is a generated file, do not edit */

export class LiftClearance_Response {
  static readonly FullTypeName = 'rmf_fleet_msgs/srv/LiftClearance_Response';

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

/*


uint32 decision
uint32 DECISION_CLEAR = 1
uint32 DECISION_CROWDED = 2

*/
