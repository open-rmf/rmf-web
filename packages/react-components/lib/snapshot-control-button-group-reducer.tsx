import React from 'react';

export type SnapshotActionFormat<T, K = undefined> = {
  type: T;
  payload: K;
};

export enum SnapshotActionType {
  AddContent = 'addContent',
  Clear = 'clear',
  Restore = 'restore',
}

export type SnapshotAction<T> =
  | SnapshotActionFormat<'addContent', T>
  | SnapshotActionFormat<'clear', React.MutableRefObject<T>>
  | SnapshotActionFormat<'restore', React.MutableRefObject<T>>;

export type SnapshotState<T> = {
  content: T;
};

function mergeContent<T extends Record<string, unknown>>(
  currentContent: T,
  storedContent: T,
  replaceCurrentContent = true,
): T {
  // In general, the constraint Record<string, XXX> doesn't actually ensure that an argument has a
  // string index signature, it merely ensures that the properties of the argument are assignable
  // to type XXX. So, in the example above you could effectively pass any object and the function
  //could write to any property without any checks. https://github.com/microsoft/TypeScript/issues/31661
  const newContents = Object.assign({}, currentContent) as Record<string, unknown>;
  Object.keys(storedContent).forEach((element: string) => {
    if (replaceCurrentContent) newContents[element] = storedContent[element];
    else {
      // If the element is already on the currentContent do not replace it
      if (!(element in currentContent)) newContents[element] = storedContent[element];
    }
  });
  return newContents as T;
}

export function snapshotReducer<T extends Record<string, unknown>>(
  state: SnapshotState<T>,
  action: SnapshotAction<T>,
): SnapshotState<T> {
  const restoreContent = (state: SnapshotState<T>, action: SnapshotAction<T>): SnapshotState<T> => {
    const newState = {
      ...state,
      content: mergeContent(state.content, action.payload.current as T, false),
    };
    action.payload.current = {} as T;
    return newState as SnapshotState<T>;
  };

  switch (action.type) {
    case SnapshotActionType.Clear:
      action.payload.current = Object.assign({}, state.content);
      return { ...state, content: {} as T };
    case SnapshotActionType.Restore:
      return restoreContent(state, action);
    case SnapshotActionType.AddContent:
      return { ...state, content: mergeContent(state.content, action.payload) };
    default:
      console.error('Unexpected action on the snapshot reducer', action);
      return state;
  }
}

interface useSnapshotReducerType<T> {
  stateSnapshot: SnapshotState<Record<string, T>>;
  dispatchSnapshot: React.Dispatch<SnapshotAction<Record<string, T>>>;
}

export function useSnapshotReducer<T>(
  initialValues: SnapshotState<Record<string, T>>,
): useSnapshotReducerType<T> {
  const [stateSnapshot, dispatchSnapshot] = React.useReducer(snapshotReducer, initialValues);
  return {
    stateSnapshot: stateSnapshot as SnapshotState<Record<string, T>>,
    dispatchSnapshot: dispatchSnapshot as React.Dispatch<SnapshotAction<Record<string, T>>>,
  };
}
