import React from 'react';
import { Typography } from '@material-ui/core';
import { Story, Meta } from '@storybook/react';
import { DashboardTooltip } from '../lib';

export default {
  title: 'Tooltip',
  component: DashboardTooltip,
} as Meta;

export const SimpleTooltip: Story = (args) => {
  return (
    <DashboardTooltip title="This is a tooltip" id="test" enabled={true} {...args}>
      <Typography variant="h5">Hover over me</Typography>
    </DashboardTooltip>
  );
};
