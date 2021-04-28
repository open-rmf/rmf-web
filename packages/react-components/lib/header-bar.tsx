import React from 'react';
import { createStyles, makeStyles, AppBar, Toolbar } from '@material-ui/core';
import { TabPanel, TabPanelProps } from './tab-panel';
import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';
import Badge from '@material-ui/core/Badge';

export interface HeaderBarProps {
  tabNames: string[];
  logoPath?: string;
  tabPanelData?: TabPanelProps[];
  children?: React.ReactNode;
}

const useStyles = makeStyles(() =>
  createStyles({
    root: {
      display: 'flex',
      flexDirection: 'row',
      alignItems: 'center',
      width: '100%',
    },
    logo: {
      maxWidth: 120,
      opacity: 1,
    },
    tabsContainer: {
      borderRight: '0.25px solid rgba(251, 252, 255, 0.5)',
      borderLeft: '0.25px solid rgba(251, 252, 255, 0.5)',
      flexGrow: 2,
    },
    toolbar: {
      textAlign: 'right',
      flexGrow: -1,
    },
  }),
);

export const HeaderBar = (props: HeaderBarProps): React.ReactElement => {
  const { logoPath, tabNames, children, tabPanelData } = props;
  const classes = useStyles();

  const [value, setValue] = React.useState(0);

  const handleChange = (event: React.ChangeEvent<unknown>, newValue: number) => {
    setValue(newValue);
  };

  const populateTabs = () => {
    const tabs = tabNames.map((tabName, index) => (
      <Tab key={index} label={tabName} value={index} />
    ));
    return tabs;
  };

  const populateTabPanels = (data: TabPanelProps[]): JSX.Element[] => {
    const tabPanels = data.map((item, idx) => {
      return (
        <TabPanel key={idx} value={value} index={idx}>
          {item.children}
        </TabPanel>
      );
    });
    return tabPanels;
  };

  return (
    <>
      <AppBar id="appbar" position="static" className={classes.root}>
        {logoPath && (
          <Tabs
            variant="standard"
            onChange={handleChange}
            value={value}
            TabIndicatorProps={{
              style: {
                display: 'none',
              },
            }}
          >
            <Tab
              label={
                <Badge badgeContent={<img src={logoPath} alt="logo" className={classes.logo} />} />
              }
            />
          </Tabs>
        )}
        <Tabs
          variant="scrollable"
          scrollButtons="auto"
          value={value}
          onChange={handleChange}
          className={classes.tabsContainer}
        >
          {populateTabs()}
        </Tabs>
        {children && (
          <Toolbar variant="dense" className={classes.toolbar}>
            {children}
          </Toolbar>
        )}
      </AppBar>
      {tabPanelData && populateTabPanels(tabPanelData)}
    </>
  );
};
