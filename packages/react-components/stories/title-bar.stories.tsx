import React from 'react';
import { Story, Meta } from '@storybook/react';
import TitleBar, { TitleBarProps } from '../lib/title-bar';
import AccountCircle from '@material-ui/icons/AccountCircle';
import IconButton from '@material-ui/core/IconButton';

export default {
  title: 'Title Bar',
  component: TitleBar,
} as Meta;

const Template: Story<TitleBarProps> = (args) => {
  return (
    <TitleBar {...args}>
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

export const SimpleTitleBar = Template.bind({});

SimpleTitleBar.args = {
  tabNames: ['Building', 'Robots', 'Tasks'],
  logoPath: '/resources/roshealth-logo-white.png',
};
