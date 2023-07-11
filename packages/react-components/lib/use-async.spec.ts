import { renderHook } from '@testing-library/react';
import { useAsync } from './use-async';

describe('useAsync', () => {
  it('resolves if component is still mounted', async () => {
    const hook = renderHook(() => useAsync());
    const safeAsync = hook.result.current;
    await expectAsync(safeAsync(Promise.resolve(true))).toBeResolvedTo(true);
  });

  it('throws if component is unmounted', async () => {
    const hook = renderHook(() => useAsync(true));
    const safeAsync = hook.result.current;
    hook.unmount();
    await expectAsync(safeAsync(Promise.resolve(true))).toBeRejected();
  });

  it('throws original error when promise is rejected', async () => {
    const hook = renderHook(() => useAsync(true));
    const safeAsync = hook.result.current;
    await expectAsync(safeAsync(Promise.reject('test error'))).toBeRejectedWith('test error');
  });

  // Commented due to the change to react 18 and because the hooks are not currently being used.
  // TODO[CR]: Eliminate before PR

  // it('referentially equal across renders', () => {
  //   const hook = renderHook(() => useAsync(true));
  //   hook.rerender();
  //   expect(hook.result.all.length).toBe(2);
  //   expect(hook.result.all[0]).toBe(hook.result.all[1]);
  // });
});
