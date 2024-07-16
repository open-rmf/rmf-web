/* This is a generated file, do not edit */

import * as rmf_task_msgs from '../../rmf_task_msgs';

export class Alert {
  static readonly FullTypeName = '';

  static readonly TIER_INFO = 0;
  static readonly TIER_WARNING = 1;
  static readonly TIER_ERROR = 2;

  id: string;
  title: string;
  subtitle: string;
  message: string;
  display: boolean;
  tier: number;
  responses_available: Array<string>;
  alert_parameters: Array<rmf_task_msgs.msg.AlertParameter>;
  task_id: string;

  constructor(fields: Partial<Alert> = {}) {
    this.id = fields.id || '';
    this.title = fields.title || '';
    this.subtitle = fields.subtitle || '';
    this.message = fields.message || '';
    this.display = fields.display || false;
    this.tier = fields.tier || 0;
    this.responses_available = fields.responses_available || [];
    this.alert_parameters = fields.alert_parameters || [];
    this.task_id = fields.task_id || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['id'] !== 'string') {
      throw new Error('expected "id" to be "string"');
    }
    if (typeof obj['title'] !== 'string') {
      throw new Error('expected "title" to be "string"');
    }
    if (typeof obj['subtitle'] !== 'string') {
      throw new Error('expected "subtitle" to be "string"');
    }
    if (typeof obj['message'] !== 'string') {
      throw new Error('expected "message" to be "string"');
    }
    if (typeof obj['display'] !== 'boolean') {
      throw new Error('expected "display" to be "boolean"');
    }
    if (typeof obj['tier'] !== 'number') {
      throw new Error('expected "tier" to be "number"');
    }
    if (!Array.isArray(obj['responses_available'])) {
      throw new Error('expected "responses_available" to be an array');
    }
    for (const [i, v] of obj['responses_available'].entries()) {
      if (typeof v !== 'string') {
        throw new Error(`expected index ${i} of "responses_available" to be "string"`);
      }
    }
    if (!Array.isArray(obj['alert_parameters'])) {
      throw new Error('expected "alert_parameters" to be an array');
    }
    for (const [i, v] of obj['alert_parameters'].entries()) {
      try {
        rmf_task_msgs.msg.AlertParameter.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "alert_parameters":\n  ` + (e as Error).message);
      }
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
  }
}

export default Alert;
