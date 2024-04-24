import { IconButton, Toolbar, Typography, styled } from '@mui/material';
import AccountCircleIcon from '@mui/icons-material/AccountCircle';
import TabContext from '@mui/lab/TabContext';
import TabPanel from '@mui/lab/TabPanel';
import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { HeaderBar } from '../lib/header-bar';
import { LogoButton } from './logo-button';
import { NavigationBar } from './navigation-bar';
import { AppBarTab } from './appbar-tab';

export default {
  title: 'Header Bar',
  component: HeaderBar,
} satisfies Meta;

export const NavBar: StoryFn = () => {
  const [value, setValue] = React.useState('building');

  const onTabChange = (event: React.ChangeEvent<unknown>, newValue: string) => {
    setValue(newValue);
  };

  return (
    <>
      <TabContext value={value}>
        <HeaderBar>
          <NavigationBar onTabChange={onTabChange} value={value}>
            <AppBarTab
              key={'building-tab'}
              label={'Building'}
              value={'building'}
              aria-label={`building-tab`}
            />
            <AppBarTab
              key={'robots'}
              label={'Robots'}
              value={'robots'}
              aria-label={`building-tab`}
            />
          </NavigationBar>
        </HeaderBar>
        <TabPanel key={'building-panel'} value={'building'}>
          <Typography variant="caption">tab panel data</Typography>
        </TabPanel>
        <TabPanel key={'robots-panel'} value={'robots'}>
          <Typography variant="caption">other tab panel data</Typography>
        </TabPanel>
      </TabContext>
    </>
  );
};

export const FullHeaderBar: StoryFn = () => {
  const classes = {
    toolbar: 'headerbar-story-toolbar',
    avatar: 'headerbar-story-avatar',
    logo: 'headerbar-story-logo',
  };
  const FullHeaderBarStory = styled('div')(({ theme }) => ({
    [`& .${classes.toolbar}`]: {
      textAlign: 'right',
      flexGrow: -1,
    },
    [`& .${classes.avatar}`]: {
      flexGrow: 1,
      minWidth: theme.spacing(16),
      overflow: 'auto',
    },
    [`& .${classes.logo}`]: {
      maxWidth: 120,
      opacity: 1,
    },
  }));

  const [value, setValue] = React.useState('building');

  const onTabChange = (event: React.ChangeEvent<unknown>, newValue: string) => {
    setValue(newValue);
  };

  return (
    <FullHeaderBarStory>
      <TabContext value={value}>
        <HeaderBar>
          <LogoButton src="/assets/roshealth-logo-white.png" />
          <NavigationBar onTabChange={onTabChange} value={value}>
            <AppBarTab
              key="building-tab"
              label="Building"
              value="building"
              aria-label="building-tab"
            />
            <AppBarTab
              key={'robots'}
              label={'Robots'}
              value={'robots'}
              aria-label={`building-tab`}
            />
          </NavigationBar>
          <Toolbar variant="dense" className={classes.toolbar}>
            <Typography variant="caption">Powered by Open-RMF</Typography>
            <IconButton id="user-btn" aria-label={'user-btn'} color="inherit">
              <AccountCircleIcon />
            </IconButton>
          </Toolbar>
        </HeaderBar>
        <TabPanel key={'building-panel'} value={'building'}>
          <Typography variant="caption">tab panel data</Typography>
        </TabPanel>
        <TabPanel key={'robots-panel'} value={'robots'}>
          <Typography variant="caption">other tab panel data</Typography>
        </TabPanel>
      </TabContext>
    </FullHeaderBarStory>
  );
};
