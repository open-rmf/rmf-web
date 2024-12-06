import { Meta, StoryObj } from '@storybook/react';

import { CustomComposeTaskForm } from './custom-compose';

export default {
  title: 'Tasks/CustomComposeTaskForm',
  component: CustomComposeTaskForm,
} satisfies Meta;

type Story = StoryObj<typeof CustomComposeTaskForm>;

export const Default: Story = {
  args: {
    taskDesc: '',
    onChange: () => {},
    onValidate: () => {},
  },
};
