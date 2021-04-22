import React from 'react';
import { Story, Meta } from '@storybook/react';
import TitleBar from '../lib/title-bar';
import AccountCircle from '@material-ui/icons/AccountCircle';
import IconButton from '@material-ui/core/IconButton';

export default {
  title: 'Title Bar',
  component: TitleBar,
} as Meta;

export const ExampleTitleBar: Story = (args) => {
  const tabNames = ['Robots', 'Tasks', 'History', 'Admin', 'Settings'];
  return (
    <TitleBar logoPath={'/resources/roshealth-logo-white.png'} tabNames={tabNames} {...args}>
      <IconButton
        aria-label="account of current user"
        aria-controls="menu-appbar"
        aria-haspopup="true"
        color="inherit"
      >
        <AccountCircle />
      </IconButton>
    </TitleBar>
  );
};
