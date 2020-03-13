import React from 'react';

export function useCombinedRefs(
  ...refs: (React.RefCallback<unknown> | React.MutableRefObject<unknown>)[]
) {
  const targetRef = React.useRef();

  React.useEffect(() => {
    refs.forEach(ref => {
      if (!ref) return;

      if (typeof ref === 'function') {
        ref(targetRef.current);
      } else {
        ref.current = targetRef.current;
      }
    });
  }, [refs]);

  return targetRef;
}
