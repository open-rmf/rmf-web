import { renderHook, act } from '@testing-library/react';
import { useStackNavigator } from '../lib';

describe('useStackNavigator', () => {
  it('should initialize with the correct initial state', () => {
    const initialState: number[] = [1, 2, 3];
    const homeView = 0;

    const { result } = renderHook(() => useStackNavigator(initialState, homeView));

    expect(result.current[0]).toEqual(initialState);
  });

  it('should push a view onto the stack', () => {
    const initialState: number[] = [1, 2];
    const homeView = 0;

    const { result } = renderHook(() => useStackNavigator(initialState, homeView));

    act(() => {
      result.current[1].push(3);
    });

    expect(result.current[0]).toEqual([1, 2, 3]);
  });

  it('should pop a view from the stack', () => {
    const initialState: number[] = [1, 2, 3];
    const homeView = 0;

    const { result } = renderHook(() => useStackNavigator(initialState, homeView));

    act(() => {
      result.current[1].pop();
    });

    expect(result.current[0]).toEqual([1, 2]);
  });

  it('should not remove the last view when popping', () => {
    const initialState: number[] = [1];
    const homeView = 0;

    const { result } = renderHook(() => useStackNavigator(initialState, homeView));

    act(() => {
      result.current[1].pop();
    });

    expect(result.current[0]).toEqual([1]);
  });

  it('should set the stack to the home view', () => {
    const initialState: number[] = [1, 2];
    const homeView = 0;

    const { result } = renderHook(() => useStackNavigator(initialState, homeView));

    act(() => {
      result.current[1].home();
    });

    expect(result.current[0]).toEqual([1, 2, 0]);
  });

  it('should reset the stack to the initial state', () => {
    const initialState: number[] = [1, 2, 3];
    const homeView = 0;

    const { result } = renderHook(() => useStackNavigator(initialState, homeView));

    act(() => {
      result.current[1].push(4);
      result.current[1].reset();
    });

    expect(result.current[0]).toEqual(initialState);
  });
});
