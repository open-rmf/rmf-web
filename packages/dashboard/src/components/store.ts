import create from 'zustand';
import { TaskState } from 'api-client';

interface Tasks {
  task: TaskState;
}

interface TaskStoreState {
  task?: TaskState[];
  setTask: (task: TaskState[]) => void;
}

export const useTaskStore = create<TaskStoreState>((set) => ({
  setTask: (task: TaskState[]) =>
    set(() => ({
      task: task,
    })),
}));
