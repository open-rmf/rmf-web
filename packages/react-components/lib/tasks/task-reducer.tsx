import React from 'react';
import type { SubmitTask } from 'api-client';
type TaskActionFormat<T, K> = {
  type: T;
  payload: K;
};

export enum TaskActionType {
  DayOfWeek = 'dayOfWeek',
  EndDatetime = 'endDatetime',
  Frequency = 'frequency',
  FrequencyType = 'frequencyType',
  FrequencyTypeCustom = 'frequencyTypeCustom',
  RuleName = 'ruleName',
  StartDatetime = 'startDatetime',
  Task = 'task',
}

export type TaskState = {
  [TaskActionType.DayOfWeek]: number[];
  [TaskActionType.EndDatetime]: string | null;
  [TaskActionType.Frequency]: number;
  [TaskActionType.FrequencyType]: string;
  [TaskActionType.FrequencyTypeCustom]: string;
  [TaskActionType.RuleName]: string;
  [TaskActionType.StartDatetime]: any;
  [TaskActionType.Task]: SubmitTask;
};

export type TaskAction =
  | TaskActionFormat<'dayOfWeek', TaskState['dayOfWeek']>
  | TaskActionFormat<'endDatetime', TaskState['endDatetime']>
  | TaskActionFormat<'frequency', TaskState['frequency']>
  | TaskActionFormat<'frequencyType', TaskState['frequencyType']>
  | TaskActionFormat<'frequencyTypeCustom', TaskState['frequencyTypeCustom']>
  | TaskActionFormat<'ruleName', TaskState['ruleName']>
  | TaskActionFormat<'startDatetime', TaskState['startDatetime']>
  | TaskActionFormat<'task', TaskState['task']>;

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
    case TaskActionType.RuleName:
      return { ...state, [TaskActionType.RuleName]: action.payload };
    case TaskActionType.StartDatetime:
      return { ...state, [TaskActionType.StartDatetime]: action.payload };
    case TaskActionType.Task:
      return { ...state, [TaskActionType.Task]: action.payload };
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
  setRuleName: (payload: TaskState['ruleName']) => void;
  setStartDatetime: (payload: TaskState['startDatetime']) => void;
  setTask: (payload: TaskState['task']) => void;
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
      setDayOfWeek: (payload) => _dispatch({ type: TaskActionType.DayOfWeek, payload: payload }),
      setEndDatetime: (payload) =>
        _dispatch({ type: TaskActionType.EndDatetime, payload: payload }),
      setFrequencyType: (payload) =>
        _dispatch({ type: TaskActionType.FrequencyType, payload: payload }),
      setFrequency: (payload) => _dispatch({ type: TaskActionType.Frequency, payload: payload }),
      setFrequencyTypeCustom: (payload) =>
        _dispatch({ type: TaskActionType.FrequencyTypeCustom, payload: payload }),
      setRuleName: (payload) => _dispatch({ type: TaskActionType.RuleName, payload: payload }),
      setStartDatetime: (payload) =>
        _dispatch({ type: TaskActionType.StartDatetime, payload: payload }),
      setTask: (payload) => _dispatch({ type: TaskActionType.Task, payload: payload }),
    } as ReducerTaskDispatch;
  }, []);
  return {
    state,
    dispatch,
  };
};
