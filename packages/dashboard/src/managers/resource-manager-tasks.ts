import { getTaskDefinition, TaskDefinition } from 'react-components';

export interface TaskResource {
  task_definition_id: string;
  display_name?: string;
}

export class TaskResourceManager {
  tasks: TaskDefinition[];

  constructor(taskResources: TaskResource[]) {
    this.tasks = [];
    for (const taskResource of taskResources) {
      const definition = getTaskDefinition(taskResource.task_definition_id);
      if (!definition) {
        console.error(
          `Invalid tasks configured for dashboard: [${taskResource.task_definition_id}]`,
        );
        continue;
      }

      if (taskResource.display_name !== undefined) {
        definition.taskDisplayName = taskResource.display_name;
      }

      this.tasks.push(definition);
    }
  }
}
