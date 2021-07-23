import React from 'react';

export type SafeAsync = <T>(maybePromise: Promise<T> | T) => Promise<T>;

export const ComponentUnmountedError = 'ComponentUnmountedError';

/**
 * Returns a function that wraps a promise to drop the chain if the component is unmounted.
 * @param throwOnUnmounted throws error if the component is unmounted when the promise resolves
 * @returns
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
