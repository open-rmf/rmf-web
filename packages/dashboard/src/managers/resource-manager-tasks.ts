export class TaskResourceManager {
  supportedTasks: string[] = ['patrol', 'delivery', 'clean'];
  taskNameRemap: Record<string, string> = {};

  constructor(
    supportedTasks: string[] | undefined,
    taskNameRemap: Record<string, string> | undefined,
  ) {
    if (supportedTasks) {
      this.supportedTasks = supportedTasks;
    }
    if (taskNameRemap) {
      this.taskNameRemap = taskNameRemap;
    }
  }
}
