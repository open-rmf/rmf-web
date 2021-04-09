/* This is a generated file, do not edit */

export class Param {
  static readonly FullTypeName = 'rmf_building_map_msgs/msg/Param';

  static readonly TYPE_UNDEFINED = 0;
  static readonly TYPE_STRING = 1;
  static readonly TYPE_INT = 2;
  static readonly TYPE_DOUBLE = 3;
  static readonly TYPE_BOOL = 4;

  name: string;
  type: number;
  value_int: number;
  value_float: number;
  value_string: string;
  value_bool: boolean;

  constructor(fields: Partial<Param> = {}) {
    this.name = fields.name || '';
    this.type = fields.type || 0;
    this.value_int = fields.value_int || 0;
    this.value_float = fields.value_float || 0;
    this.value_string = fields.value_string || '';
    this.value_bool = fields.value_bool || false;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (typeof obj['type'] !== 'number') {
      throw new Error('expected "type" to be "number"');
    }
    if (typeof obj['value_int'] !== 'number') {
      throw new Error('expected "value_int" to be "number"');
    }
    if (typeof obj['value_float'] !== 'number') {
      throw new Error('expected "value_float" to be "number"');
    }
    if (typeof obj['value_string'] !== 'string') {
      throw new Error('expected "value_string" to be "string"');
    }
    if (typeof obj['value_bool'] !== 'boolean') {
      throw new Error('expected "value_bool" to be "boolean"');
    }
  }
}

/*
string name

uint32 type
uint32 TYPE_UNDEFINED=0
uint32 TYPE_STRING=1
uint32 TYPE_INT=2
uint32 TYPE_DOUBLE=3
uint32 TYPE_BOOL=4

int32 value_int
float32 value_float
string value_string
bool value_bool

*/
