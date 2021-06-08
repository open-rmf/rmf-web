import React from 'react';
import { Story, Meta } from '@storybook/react';
import { CircularProgressBar as CircularProgressBar_, CircularProgressBarProps } from '../../lib';
import { Typography } from '@material-ui/core';

export default {
  title: 'Robots/Circular Progress Bar',
  component: CircularProgressBar_,
} as Meta;

export const CircularProgressBar: Story<CircularProgressBarProps> = (args) => {
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
