import { Typography } from '@mui/material';
import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import {
  CircularProgressBar as CircularProgressBar_,
  CircularProgressBarProps,
} from './circular-progress-bar';

export default {
  title: 'Robots/Circular Progress Bar',
  component: CircularProgressBar_,
} satisfies Meta;

export const CircularProgressBar: StoryFn<CircularProgressBarProps> = (args) => {
  return (
    <>
      <CircularProgressBar_ {...args}>
        <Typography variant="h6">{args.progress}%</Typography>
      </CircularProgressBar_>
    </>
  );
};

CircularProgressBar.args = {
  progress: 70,
  strokeColor: 'green',
};
