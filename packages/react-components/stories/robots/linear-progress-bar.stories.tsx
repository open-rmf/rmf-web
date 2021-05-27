import React from 'react';
import { Story, Meta } from '@storybook/react';
import { LinearProgressBar as LinearProgressBar_ } from '../../lib';
import { LinearProgressProps } from '@material-ui/core';

export default {
  title: 'Robots/Linear Progress Bar',
  component: LinearProgressBar_,
} as Meta;

export const LinearProgressBar: Story<LinearProgressProps & { value: number }> = (args) => {
  return <LinearProgressBar_ {...args} />;
};

LinearProgressBar.args = {
  value: 100,
};
