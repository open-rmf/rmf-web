export type SnapshotActionFormat<T, K = undefined> = {
  type: T;
  payload: K;
};

export enum SnapshotActionType {
  ADD_CONTENT = 'addContent',
  CLEAR = 'clear',
  RESTORE = 'restore',
}

export enum SnapshotStateType {
  CONTENT = 'content',
}

export type SnapshotAction =
  | SnapshotActionFormat<'addContent', any>
  | SnapshotActionFormat<'clear', any>
  | SnapshotActionFormat<'restore', any>;

export type SnapshotState = {
  [SnapshotStateType.CONTENT]: any;
};

function mergeContent(
  currentContent: any,
  storedContent: any,
  replaceCurrentContent: boolean = true,
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
    case SnapshotActionType.CLEAR:
      action.payload.stored = Object.assign({}, state[SnapshotStateType.CONTENT]);
      return { ...state, [SnapshotStateType.CONTENT]: {} };
    case SnapshotActionType.RESTORE:
      const mergedContent = mergeContent(state.content, action.payload.stored, false);
      const newState = { ...state, [SnapshotStateType.CONTENT]: mergedContent };
      action.payload.stored = {};
      return newState;
    case SnapshotActionType.ADD_CONTENT:
      return { ...state, [SnapshotStateType.CONTENT]: mergeContent(state.content, action.payload) };
    default:
      console.error('Unexpected action');
      return state;
  }
};
