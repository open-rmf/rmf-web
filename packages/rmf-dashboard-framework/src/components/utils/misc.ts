import type { Time } from 'api-client';
import shallowEqual from 'shallowequal';

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
export function rosTimeToJs(rosTime: Time): Date {
  return new Date(rosTime.sec * 1000 + Math.floor(rosTime.nanosec / 1000000));
}

let id = 0;
export function uniqueId(): string {
  return (id++).toString();
}

/**
 * Performs shallow equal comparison on 2 objects, except the keys in `into` are traversed down.
 * @param into
 */
export function almostShallowEqual<T>(objA: T, objB: T, into: (keyof T)[] = []): boolean {
  return shallowEqual(objA, objB, (a, b, k) => {
    if (k && into.includes(k as keyof T)) {
      return shallowEqual(a, b);
    }
    return undefined;
  });
}
