import { Meta, Story } from '@storybook/react';
import type { SubmitTask } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { CreateTaskForm, CreateTaskFormProps } from '../../lib';

export default {
  title: 'Tasks/Create Task',
  component: CreateTaskForm,
} as Meta;

function makeTask() {
  return {
    description: {
      cleaning_zone: 'zone',
    },
    start_time: Math.floor(Date.now() / 1000),
    task_type: RmfModels.TaskType.TYPE_CLEAN,
    priority: 0,
  };
}

export const CreateTask: Story<CreateTaskFormProps> = (args) => {
  const [tasks, setTasks] = React.useState<SubmitTask[]>([]);
  const [selectedTask, setSelectedTask] = React.useState(0);
  const handleUploadFileClick: CreateTaskFormProps['onUploadFileClick'] = () => {
    const uploadedTasks: SubmitTask[] = [];
    for (let i = 0; i < 100; i++) {
      uploadedTasks.push(makeTask());
    }
    setTasks(uploadedTasks);
  };
  return (
    <CreateTaskForm
      {...args}
      open
      tasks={tasks}
      selectedTaskIdx={selectedTask}
      onTasksChange={setTasks}
      onUploadFileClick={handleUploadFileClick}
      onSelectTask={setSelectedTask}
    ></CreateTaskForm>
  );
};

CreateTask.args = {
  submitTasks: async () => new Promise((res) => setTimeout(res, 1000)),
  cleaningZones: ['test_zone_0', 'test_zone_1'],
  loopWaypoints: ['test_waypoint_0', 'test_waypoint_1'],
  deliveryWaypoints: ['test_waypoint_0', 'test_waypoint_1'],
  dispensers: ['test_dispenser_0', 'test_dispenser_1'],
  ingestors: ['test_ingestor_0', 'test_ingestor_1'],
};
