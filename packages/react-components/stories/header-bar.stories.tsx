import React from 'react';
import { Story, Meta } from '@storybook/react';
import { HeaderBar, HeaderBarProps } from '../lib/header-bar';
import { IconButton, Typography } from '@material-ui/core';
import AccountCircleIcon from '@material-ui/icons/AccountCircle';

export default {
  title: 'Header Bar',
  component: HeaderBar,
} as Meta;

const Template: Story<HeaderBarProps> = (args) => {
  return <HeaderBar {...args} />;
};

export const NavigationBar = Template.bind({});
NavigationBar.args = {
  tabNames: ['Home', '7-Day History', 'Archive'],
};

export const SimpleHeaderBar = Template.bind({});

SimpleHeaderBar.args = {
  tabNames: ['Building', 'Robots', 'Tasks'],
  logoPath: '/resources/roshealth-logo-white.png',
};

export const DetailedHeaderBar: Story<HeaderBarProps> = (args) => {
  interface TabInfo {
    value: number;
    index: number;
    children: React.ReactNode;
  }

  const createItems = (n: number) => {
    const itemArray: TabInfo[] = [];
    for (let i = 0; i < n; i++) {
      itemArray.push({
        value: i,
        index: i,
        children: [
          <Typography variant="h6" key={0}>
            item {i}
          </Typography>,
        ],
      });
    }
    return itemArray;
  };

  return (
    <>
      <HeaderBar tabPanelData={createItems(3)} {...args}>
        <Typography variant="caption">Powered by OpenRMF</Typography>
        <IconButton id="user-btn" aria-label={'user-btn'} color="inherit">
          <AccountCircleIcon />
        </IconButton>
      </HeaderBar>
    </>
  );
};

DetailedHeaderBar.args = {
  tabNames: ['Building', 'Robots', 'Tasks'],
  logoPath: '/resources/roshealth-logo-white.png',
};
