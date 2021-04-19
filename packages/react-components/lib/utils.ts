import * as RmfModels from 'rmf-models';

/**
 * Create an object that automatically creates a value if it doesn't exist.
 */
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export function defaultDict<T>(factory: (key: any) => T): Record<any, T> {
  const records: Record<string, T> = {};
  return new Proxy(records, {
    get: (target, prop: string) => {
      if (!(prop in target)) {
        target[prop] = factory(prop);
      }
      return target[prop];
    },
  });
}

/**
 * Converts a ROS time representation to a javascript Date object.
 * The timezone is assumed to be UTC.
 * The precision will be reduced to milliseconds.
 */
export function rosTimeToJs(rosTime: RmfModels.Time): Date {
  return new Date(rosTime.sec * 1000 + Math.floor(rosTime.nanosec / 1000000));
}
