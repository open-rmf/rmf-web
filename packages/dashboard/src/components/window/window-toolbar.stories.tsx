import { IconButton } from '@mui/material';
import { Meta, StoryObj } from '@storybook/react';

import OpenInFullIcon from '../icons/OpenInFull';
import { WindowToolbar } from './window-toolbar';

export default {
  title: 'Window/Toolbar',
  component: WindowToolbar,
  argTypes: {
    title: {
      defaultValue: 'Example',
    },
  },
} satisfies Meta;

type Story = StoryObj<typeof WindowToolbar>;

export const Toolbar: Story = {
  render: (args) => (
    <WindowToolbar {...args}>
      <IconButton color="inherit">
        <OpenInFullIcon />
      </IconButton>
    </WindowToolbar>
  ),
};
