import { IconButton } from '@mui/material';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import OpenInFullIcon from '../icons/OpenInFull';
import { WindowToolbar, WindowToolbarProps } from './window-toolbar';

export default {
  title: 'Window/Toolbar',
  component: WindowToolbar,
  argTypes: {
    title: {
      defaultValue: 'Example',
    },
  },
} as Meta;

export const Toolbar: Story<WindowToolbarProps> = (args) => {
  return (
    <WindowToolbar {...args}>
      <IconButton color="inherit">
        <OpenInFullIcon />
      </IconButton>
    </WindowToolbar>
  );
};
