/* This is a generated file, do not edit */

import { AlertParameter } from '../../rmf_task_msgs/msg/AlertParameter';

export class Alert {
  static readonly FullTypeName = 'rmf_task_msgs/msg/Alert';

  static readonly TIER_INFO = 0;
  static readonly TIER_WARNING = 1;
  static readonly TIER_ERROR = 2;

  id: string;
  title: string;
  subtitle: string;
  message: string;
  display: boolean;
  tier: number;
  responses_available: string[];
  alert_parameters: AlertParameter[];
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
        AlertParameter.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "alert_parameters":\n  ` + (e as Error).message);
      }
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
  }
}

/*
# The unique ID which responses can reply to
string id

# Title, subtitle and message to be displayed on any frontend
string title
string subtitle
string message

# Whether this alert should be displayed on any frontend, default
# as true
bool display true

# The severity tier of this alert
uint8 tier
uint8 TIER_INFO=0
uint8 TIER_WARNING=1
uint8 TIER_ERROR=2

# Responses available for this alert. If no responses are expected
# this field can be left empty
string[] responses_available

# Parameters that may be useful for custom interactions
AlertParameter[] alert_parameters

# The task ID that is involved in this alert. If no task is involved
# this string can be left empty
string task_id

*/
