import { IconButton } from '@mui/material';
import { Meta, StoryFn } from '@storybook/react';
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
} satisfies Meta;

export const Toolbar: StoryFn<WindowToolbarProps> = (args: WindowToolbarProps) => {
  return (
    <WindowToolbar {...args}>
      <IconButton color="inherit">
        <OpenInFullIcon />
      </IconButton>
    </WindowToolbar>
  );
};
