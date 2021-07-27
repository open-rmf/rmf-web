/**
 * Helper function to create a callback that returns a map of callbacks from a Record. Meant to be
 * used as the callback for `React.useMemo` to create callbacks that match equality test at each
 * render.
 *
 * Given this render
 * ```tsx
 * {items.map((i) => <ListItem onClick={doSomething(i.value)} />}
 * ```
 * This looks fine at first but this causes problems when using `React.memo` because at each render
 * new functions are created which doesn't equal to the previous (functions in js are only equal
 * to itself, identical functions are never equal).
 *
 * The naive approach would be to use `React.useCallback` but that hits another problem, you can't
 * use hooks in a callback and the `ListItem` is generated in the `map` callback.
 *
 * ```tsx
 * /// doesn't work!
 * {items.map((i) => <ListItem onClick={React.useCallback(() => doSomething(i.value))} />}
 * ```
 *
 * So then how about using `React.useCallback` outside the `map` function? Then you lost your
 * reference to `i`.
 *
 * ```tsx
 * // `i` not defined!
 * React.useCallback(() => doSomething(i.value));
 * ```
 *
 * So then the solution is to create an array of callbacks, one for each item and wrap them in
 * `React.useMemo`, this way the same callback is used each render and each callback can receive
 * a reference to the `i` object.
 *
 * ```tsx
 * // ok!
 * React.useMemo(() => items.map((i) => callback(i, ...onClickArgs)));
 * ```
 *
 * @example
 * ```ts
 *   const onClick = React.useMemo(
 *   makeCallbackMapCallback<Required<DoorItemProps>['onClick']>(
 *    doorsMap,
 *    (_, door) => onDoorClick && onDoorClick(door),
 *  ),
 *  [doorsMap, onDoorClick],
 * );
 * ```
 *
 * @param arr
 * @param callback
 */
export function makeCallbackArrayCallback<
  U extends (...args: any[]) => any = (...args: any[]) => any,
  T = any,
  Args extends any[] = U extends (...args: infer Args) => any ? Args : never,
  R = U extends (...args: any[]) => infer R ? R : never
>(arr: T[], callback: (item: T, ...args: Args) => R): () => ((...args: Args) => R)[] {
  return () => arr.map((item) => (...args: Args) => callback(item, ...args));
}
