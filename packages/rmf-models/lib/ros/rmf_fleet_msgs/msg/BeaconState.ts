/* This is a generated file, do not edit */

export class BeaconState {
  static readonly FullTypeName = '';

  id: string;
  online: boolean;
  category: string;
  activated: boolean;
  level: string;

  constructor(fields: Partial<BeaconState> = {}) {
    this.id = fields.id || '';
    this.online = fields.online || false;
    this.category = fields.category || '';
    this.activated = fields.activated || false;
    this.level = fields.level || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['id'] !== 'string') {
      throw new Error('expected "id" to be "string"');
    }
    if (typeof obj['online'] !== 'boolean') {
      throw new Error('expected "online" to be "boolean"');
    }
    if (typeof obj['category'] !== 'string') {
      throw new Error('expected "category" to be "string"');
    }
    if (typeof obj['activated'] !== 'boolean') {
      throw new Error('expected "activated" to be "boolean"');
    }
    if (typeof obj['level'] !== 'string') {
      throw new Error('expected "level" to be "string"');
    }
  }
}

export default BeaconState;
