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
