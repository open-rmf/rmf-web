import React from 'react';
import { Story, Meta } from '@storybook/react';
import { ProgressBar as ProgressBar_ } from '../lib/progressbar';
import { LinearProgressProps } from '@material-ui/core';

export default {
  title: 'Progress Bar',
  component: ProgressBar_,
} as Meta;

export const ProgressBar: Story<LinearProgressProps & { value: number }> = (args) => {
  return <ProgressBar_ {...args} />;
};

ProgressBar.args = {
  value: 100,
};

export const SecondaryProgressBar: Story<LinearProgressProps & { value: number }> = (args) => {
  return <ProgressBar_ {...args} />;
};

SecondaryProgressBar.args = {
  value: 50,
  color: 'secondary',
};
