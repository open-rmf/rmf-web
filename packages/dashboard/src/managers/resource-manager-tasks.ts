import { getDefaultTaskDefinition, TaskDefinition } from 'react-components';

export interface TaskResource {
  task_definition_id: string;
  display_name?: string;
}

export class TaskResourceManager {
  tasks: TaskDefinition[];

  constructor(taskResources: TaskResource[]) {
    this.tasks = [];
    for (const taskResource of taskResources) {
      const defaultTaskDefinition = getDefaultTaskDefinition(taskResource.task_definition_id);
      if (!defaultTaskDefinition) {
        console.error(
          `Invalid tasks configured for dashboard: [${taskResource.task_definition_id}]`,
        );
        continue;
      }

      if (taskResource.display_name !== undefined) {
        this.tasks.push({
          ...defaultTaskDefinition,
          taskDisplayName: taskResource.display_name,
        });
      } else {
        this.tasks.push(defaultTaskDefinition);
      }
    }
  }
}
