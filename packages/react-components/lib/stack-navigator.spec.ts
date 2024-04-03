import { act, renderHook, RenderHookResult } from '@testing-library/react';
import { StackNavigatorDispatch, useStackNavigator } from '../lib';

describe('useStackNavigator', () => {
  let hookResult: RenderHookResult<
    ReturnType<typeof useStackNavigator<number>>,
    Parameters<typeof useStackNavigator<number>>
  >['result'];
  let stackDispatch: StackNavigatorDispatch<number>;

  beforeEach(() => {
    hookResult = renderHook(() => useStackNavigator([0], 0)).result;
    stackDispatch = hookResult.current[1];
  });

  it('push', () => {
    act(() => stackDispatch.push(10));
    const stack = hookResult.current;
    expect(stack).toHaveLength(2);
    expect(stack[0][0]).toBe(0);
    expect(stack[0][1]).toBe(10);
  });

  it('pop does not remove last item', () => {
    act(() => {
      stackDispatch.push(2);
      stackDispatch.pop();
      stackDispatch.pop();
    });
    const stack = hookResult.current[0];
    expect(stack).toHaveLength(1);
    expect(stack[0]).toBe(0);
  });

  it('reset returns the stack to the initial state', () => {
    act(() => {
      stackDispatch.push(2);
      stackDispatch.push(3);
    });
    expect(hookResult.current[0]).toHaveLength(3);
    act(() => {
      stackDispatch.reset();
    });
    expect(hookResult.current[0]).toHaveLength(1);
    expect(hookResult.current[0][0]).toBe(0);
  });

  it('home pushes the home view onto the stack', () => {
    act(() => {
      stackDispatch.push(2);
      stackDispatch.push(3);
    });
    expect(hookResult.current[0]).toHaveLength(3);
    act(() => {
      stackDispatch.home();
    });
    expect(hookResult.current[0]).toHaveLength(4);
    expect(hookResult.current[0][0]).toBe(0);
  });
});
