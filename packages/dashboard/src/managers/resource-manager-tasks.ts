import { TaskDefinition } from 'react-components';

export class TaskResourceManager {
  supportedTasks: Record<string, TaskDefinition>;

  constructor(supportedTasks: TaskDefinition[] | undefined) {
    this.supportedTasks = {};
    if (supportedTasks) {
      for (const t of supportedTasks) {
        this.supportedTasks[t.taskDefinitionId] = t;
      }
    }
  }
}
