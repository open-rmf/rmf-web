import { LinearProgressProps } from '@mui/material';
import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { LinearProgressBar as LinearProgressBar_ } from './linear-progress-bar';

export default {
  title: 'Robots/Linear Progress Bar',
  component: LinearProgressBar_,
} satisfies Meta;

export const LinearProgressBar: StoryFn<LinearProgressProps & { value: number }> = (args) => {
  return <LinearProgressBar_ {...args} />;
};

LinearProgressBar.args = {
  value: 100,
};
