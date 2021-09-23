import { IconButton, Toolbar, Typography, TabProps, styled } from '@material-ui/core';
import Tab from '@material-ui/core/Tab';
import AccountCircleIcon from '@material-ui/icons/AccountCircle';
import TabContext from '@material-ui/lab/TabContext';
import TabPanel from '@material-ui/lab/TabPanel';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { HeaderBar } from '../lib/header-bar';
import { LogoButton } from './logo-button';
import { NavigationBar } from './navigation-bar';

export default {
  title: 'Header Bar',
  component: HeaderBar,
} as Meta;

const StyledTab = styled((props: TabProps) => <Tab {...props} />)(({ theme }) => ({
  color: 'rgba(255, 255, 255, 0.7)',
  '&.Mui-selected': {
    color: theme.palette.text.primary,
  },
  '&.Mui-focusVisible': {
    backgroundColor: 'rgba(100, 95, 228, 0.32)',
  },
}));

export const NavBar: Story = () => {
  const [value, setValue] = React.useState('building');

  const onTabChange = (event: React.ChangeEvent<unknown>, newValue: string) => {
    setValue(newValue);
  };

  return (
    <>
      <TabContext value={value}>
        <HeaderBar>
          <NavigationBar onTabChange={onTabChange} value={value}>
            <StyledTab
              key={'building-tab'}
              label={'Building'}
              value={'building'}
              aria-label={`building-tab`}
            />
            <StyledTab
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

export const FullHeaderBar: Story = () => {
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
            <StyledTab
              key="building-tab"
              label="Building"
              value="building"
              aria-label="building-tab"
            />
            <StyledTab
              key={'robots'}
              label={'Robots'}
              value={'robots'}
              aria-label={`building-tab`}
            />
          </NavigationBar>
          <Toolbar variant="dense" className={classes.toolbar}>
            <Typography variant="caption">Powered by OpenRMF</Typography>
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
