import { Meta, Story } from '@storybook/react';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { TaskPhases as TaskPhases_, TaskPhasesProps } from '../../lib';

const task: RmfModels.TaskSummary = new RmfModels.TaskSummary({
  end_time: { sec: 0, nanosec: 0 },
  start_time: { sec: 0, nanosec: 0 },
  state: RmfModels.TaskSummary.STATE_ACTIVE,
  status: `Phase 1:
test phase 1

Phase 2:
test phase 2

*Phase 3:
test phase 3`,
  submission_time: { sec: 0, nanosec: 500 },
  task_id: '8b49e999-d246-4395-80f7-ebc5ca85e639',
});

export default {
  title: 'Tasks/Task Phases',
  component: TaskPhases_,
  argTypes: {
    taskSummary: {
      control: {
        type: 'object',
      },
    },
  },
} as Meta;

export const TaskPhases: Story<TaskPhasesProps> = (args) => {
  return <TaskPhases_ {...args}></TaskPhases_>;
};

TaskPhases.args = {
  taskSummary: task,
};
