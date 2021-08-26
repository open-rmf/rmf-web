import React from 'react';

type TaskActionFormat<T, K> = {
  type: T;
  payload: K;
};

export enum TaskActionType {
  FrequencyType = 'frequencyType',
  Frequency = 'frequency',
  FrequencyTypeCustom = 'frequencyTypeCustom',
  DayOfWeek = 'dayOfWeek',
  EndDatetime = 'endDatetime',
}

export type TaskState = {
  [TaskActionType.DayOfWeek]: number[];
  [TaskActionType.EndDatetime]: string | null;
  [TaskActionType.Frequency]: number;
  [TaskActionType.FrequencyType]: string;
  [TaskActionType.FrequencyTypeCustom]: string;
};

export type TaskAction =
  | TaskActionFormat<'dayOfWeek', TaskState['dayOfWeek']>
  | TaskActionFormat<'endDatetime', TaskState['endDatetime']>
  | TaskActionFormat<'frequency', TaskState['frequency']>
  | TaskActionFormat<'frequencyType', TaskState['frequencyType']>
  | TaskActionFormat<'frequencyTypeCustom', TaskState['frequencyTypeCustom']>;

export const taskReducer = (state: TaskState, action: TaskAction): TaskState => {
  switch (action.type) {
    case TaskActionType.DayOfWeek:
      return { ...state, [TaskActionType.DayOfWeek]: action.payload };
    case TaskActionType.EndDatetime:
      return { ...state, [TaskActionType.EndDatetime]: action.payload };
    case TaskActionType.Frequency:
      return { ...state, [TaskActionType.Frequency]: action.payload };
    case TaskActionType.FrequencyType:
      return { ...state, [TaskActionType.FrequencyType]: action.payload };
    case TaskActionType.FrequencyTypeCustom:
      return { ...state, [TaskActionType.FrequencyTypeCustom]: action.payload };

    default:
      console.error('Unexpected action');
      return state;
  }
};

export interface ReducerTaskDispatch {
  setDayOfWeek: (payload: TaskState['dayOfWeek']) => void;
  setEndDatetime: (payload: TaskState['endDatetime']) => void;
  setFrequency: (payload: TaskState['frequency']) => void;
  setFrequencyType: (payload: TaskState['frequencyType']) => void;
  setFrequencyTypeCustom: (payload: TaskState['frequencyTypeCustom']) => void;
}

export interface ReducerTaskProps {
  state: TaskState;
  dispatch: ReducerTaskDispatch;
}

export const useTaskReducer = (initialValue: TaskState): ReducerTaskProps => {
  const [_state, _dispatch] = React.useReducer(taskReducer, initialValue);
  // We add a useMemo here because React identifies that the state and dispatch props are always
  // changing, which causes a huge performance issue.
  const state = React.useMemo(() => _state, [_state]);
  const dispatch: ReducerTaskDispatch = React.useMemo(() => {
    return {
      setFrequencyType: (payload) =>
        _dispatch({ type: TaskActionType.FrequencyType, payload: payload }),
      setFrequency: (payload) => _dispatch({ type: TaskActionType.Frequency, payload: payload }),
      setFrequencyTypeCustom: (payload) =>
        _dispatch({ type: TaskActionType.FrequencyTypeCustom, payload: payload }),
      setDayOfWeek: (payload) => _dispatch({ type: TaskActionType.DayOfWeek, payload: payload }),
      setEndDatetime: (payload) =>
        _dispatch({ type: TaskActionType.EndDatetime, payload: payload }),
    } as ReducerTaskDispatch;
  }, []);
  return {
    state,
    dispatch,
  };
};
