import { Typography } from '@mui/material';
import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { Tooltip } from './tooltip';

export default {
  title: 'Tooltip',
  component: Tooltip,
} satisfies Meta;

export const SimpleTooltip: StoryFn = (args) => {
  return (
    <Tooltip title="This is a tooltip" id="test" enabled={true} {...args}>
      <Typography variant="h5">Hover over me</Typography>
    </Tooltip>
  );
};
