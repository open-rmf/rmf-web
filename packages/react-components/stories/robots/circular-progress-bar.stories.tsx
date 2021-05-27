import React from 'react';
import { Story, Meta } from '@storybook/react';
import {
  CircularProgressBar,
  CircularProgressBarProps,
} from '../../lib/robots/circular-progress-bar';
import { Typography } from '@material-ui/core';

export default {
  title: 'Circular Progress Bar',
  component: CircularProgressBar,
} as Meta;

export const ActiveProgressBar: Story<CircularProgressBarProps> = (args) => {
  return (
    <>
      <CircularProgressBar {...args}>
        <Typography variant="h6">{args.progress}%</Typography>
      </CircularProgressBar>
    </>
  );
};

ActiveProgressBar.args = {
  progress: 70,
  strokeColor: 'green',
};
