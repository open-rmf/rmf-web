import * as RomiCore from '@osrf/romi-js-core-interfaces';

class Emergency {
  static readonly typeName = 'std_msgs/msg/Bool';
  static fromObject(obj: any): Emergency {
    if (typeof obj.data !== 'boolean') throw Error('data must be of type boolean');
    return new Emergency(obj.data);
  }
  constructor(public data: boolean) {}
}

export const emergency: RomiCore.RomiTopic<Emergency> = {
  validate: (msg) => Emergency.fromObject(msg),
  type: Emergency.typeName,
  topic: 'fire_alarm_trigger',
};
