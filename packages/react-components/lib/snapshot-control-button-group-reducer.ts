export type SnapshotActionFormat<T, K = undefined> = {
  type: T;
  payload: K;
};

export enum SnapshotActionType {
  AddContent = 'addContent',
  Clear = 'clear',
  Restore = 'restore',
}

export enum SnapshotStateType {
  Content = 'content',
}

export type SnapshotAction =
  | SnapshotActionFormat<'addContent', any>
  | SnapshotActionFormat<'clear', any>
  | SnapshotActionFormat<'restore', any>;

export type SnapshotState = {
  [SnapshotStateType.Content]: any;
};

function mergeContent(
  currentContent: any,
  storedContent: any,
  replaceCurrentContent = true,
) {
  const newContents = Object.assign({}, currentContent);
  Object.keys(storedContent).forEach((element) => {
    if (replaceCurrentContent) newContents[element] = storedContent[element];
    else {
      // If the element is already on the currentContent do not replace it
      if (!(element in currentContent)) newContents[element] = storedContent[element];
    }
  });
  return newContents;
}

export const snapshotReducer = (state: SnapshotState, action: SnapshotAction): SnapshotState => {
  switch (action.type) {
    case SnapshotActionType.Clear:
      action.payload.stored = Object.assign({}, state[SnapshotStateType.Content]);
      return { ...state, [SnapshotStateType.Content]: {} };
    case SnapshotActionType.Restore:
      const mergedContent = mergeContent(state.content, action.payload.stored, false);
      const newState = { ...state, [SnapshotStateType.Content]: mergedContent };
      action.payload.stored = {};
      return newState;
    case SnapshotActionType.AddContent:
      return { ...state, [SnapshotStateType.Content]: mergeContent(state.content, action.payload) };
    default:
      console.error('Unexpected action');
      return state;
  }
};
