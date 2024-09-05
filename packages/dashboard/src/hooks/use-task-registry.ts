import { TaskDefinition } from 'react-components';

import { createDeferredContext } from './deferred-context';

export interface TaskRegistry {
  /**
   * List of tasks that can be submitted.
   */
  taskDefinitions: TaskDefinition[];

  /**
   * List of available pickup zones used for delivery tasks.
   */
  pickupZones: string[];

  // FIXME(koonpeng): this is used for very specific tasks, should be removed when mission
  // system is implemented.
  cartIds: string[];
}

export const [useTaskRegistry, TaskRegistryProvider] = createDeferredContext<TaskRegistry>();
