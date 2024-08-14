import { Typography } from '@mui/material';
import { Meta, StoryObj } from '@storybook/react';

import { Tooltip } from './tooltip';

export default {
  title: 'Tooltip',
  component: Tooltip,
} satisfies Meta;

type Story = StoryObj<typeof Tooltip>;

export const SimpleTooltip: Story = {
  args: {
    title: 'This is a tooltip',
  },
  render: (args) => (
    <Tooltip {...args} id="test" enabled={true}>
      <Typography variant="h5">Hover over me</Typography>
    </Tooltip>
  ),
};
