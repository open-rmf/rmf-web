import React from 'react';

export type SafeAsync = <T>(maybePromise: Promise<T> | T) => Promise<T>;

export const ComponentUnmountedError = 'ComponentUnmountedError';

/**
 * Returns a function that wraps a promise to drop the chain if the component is unmounted.
 * @param throwOnUnmounted throws error if the component is unmounted when the promise resolves
 * @returns
 *
 * @remarks This way this works is that it
 * 1. Creates a `useRef` hook.
 * 2. Creates a `useEffect` hook with no actions and no dependencies, but with a cleanup
 *   function that sets the ref to `false`. Because there is no dependencies, the cleanup
 *   function will only run when the component is unmounted.
 * Then it returns a callback with the signature `(wrappedPromise: Promise) => Promise`,
 *   that returns a new promise (`wrapperPromise`) that:
 * 1. awaits on the original promise.
 * 2. after the promise is resolved, it checks the ref to see if the component is unmounted.
 *   2.1 if the component is still mounted, resolves `wrapperPromise` with the result of
 *     `wrappedPromise`.
 *   2.2 If the component is unmounted and `throwOnUnmounted` is `false`,
 *     `wrappedPromise` NEVER resolves or rejects, so statements after the `await` never runs.
 *       2.2.1 If `throwOnUnomunted` is `true`, `wrappedPromises` is rejected with an error.
 *
 * Example:
 * ```tsx
 * async function fetchData() { ... }
 *
 * function Foo() {
 *   const safeAsync = useAsync();
 *   const [data, setData] = React.useState(...);
 *   React.useEffect(() => {
 *     (async () => {
 *       const newData = await safeAsync(fetchData());
 *       setData(newData); // will not run if component is already unmounted.
 *
 *       // works when everything is in 1 statement also.
 *       // setData(await safeAsync(fetchData()));
 *     })();
 *   }, []);
 *
 *   return ...;
 * }
 * ```
 *
 */
export function useAsync(throwOnUnmounted = false): SafeAsync {
  const mountedRef = React.useRef(true);

  React.useEffect(
    () => () => {
      mountedRef.current = false;
    },
    [],
  );

  return React.useCallback(
    (maybePromise) => {
      if (!(maybePromise instanceof Promise)) {
        return Promise.resolve(maybePromise);
      }
      return new Promise((res, rej) => {
        maybePromise
          .then((result) => {
            if (!mountedRef.current) {
              if (throwOnUnmounted) {
                const error = new Error('component is unmounted or promise was cancelled');
                error.name = ComponentUnmountedError;
                throw error;
              }
            } else {
              res(result);
            }
          })
          .catch(rej);
      });
    },
    [throwOnUnmounted],
  );
}
