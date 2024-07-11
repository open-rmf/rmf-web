import { renderHook } from '@testing-library/react';
import { useAsync } from './use-async';

describe('useAsync', () => {
  it('resolves if component is still mounted', async () => {
    const hook = renderHook(() => useAsync());
    const safeAsync = hook.result.current;
    await expect(safeAsync(Promise.resolve(true))).resolves.toBe(true);
  });

  it('throws if component is unmounted', async () => {
    const hook = renderHook(() => useAsync(true));
    const safeAsync = hook.result.current;
    hook.unmount();
    await expect(safeAsync(Promise.resolve(true))).rejects.toThrow();
  });

  it('throws original error when promise is rejected', async () => {
    const hook = renderHook(() => useAsync(true));
    const safeAsync = hook.result.current;
    await expect(safeAsync(Promise.reject('test error'))).rejects.toBe('test error');
  });

  it('referentially equal across renders', async () => {
    const hook = renderHook(() => useAsync(true));
    const first = hook.result.current;
    hook.rerender();
    expect(hook.result.current).toBe(first);
  });
});
