/* This is a generated file, do not edit */

export class GetBuildingMap_Request {
  static readonly FullTypeName = '';

  structure_needs_at_least_one_member: number;

  constructor(fields: Partial<GetBuildingMap_Request> = {}) {
    this.structure_needs_at_least_one_member = fields.structure_needs_at_least_one_member || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['structure_needs_at_least_one_member'] !== 'number') {
      throw new Error('expected "structure_needs_at_least_one_member" to be "number"');
    }
  }
}

export default GetBuildingMap_Request;
