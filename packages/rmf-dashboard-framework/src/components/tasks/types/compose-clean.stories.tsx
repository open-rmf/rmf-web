import { Meta, StoryObj } from '@storybook/react';

import { ComposeCleanTaskForm, makeDefaultComposeCleanTaskDescription } from './compose-clean';

export default {
  title: 'Tasks/ComposeCleanTaskForm',
  component: ComposeCleanTaskForm,
} satisfies Meta;

type Story = StoryObj<typeof ComposeCleanTaskForm>;

export const Default: Story = {
  args: {
    taskDesc: makeDefaultComposeCleanTaskDescription(),
    cleaningZones: ['clean_zone_1', 'clean_zone_2'],
    onChange: () => {},
    onValidate: () => {},
  },
};
