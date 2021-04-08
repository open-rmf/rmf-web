import React from 'react';
import { makeStyles, Theme } from '@material-ui/core/styles';
import AppBar from '@material-ui/core/AppBar';
import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';
import Box from '@material-ui/core/Box';

interface TabPanelProps {
  children?: React.ReactNode;
  index: number;
  value: number;
}

function TabPanel(props: TabPanelProps) {
  const { children, value, index, ...other } = props;

  return (
    <div
      role="tabpanel"
      hidden={value !== index}
      id={`navigation-tabpanel-${index}`}
      aria-labelledby={`navigation-tab-${index}`}
      {...other}
    >
      {value === index && <Box p={3}>{children}</Box>}
    </div>
  );
}

const useStyles = makeStyles((theme: Theme) => ({
  root: {
    flexGrow: 1,
    backgroundColor: theme.palette.background.paper,
  },
}));

interface NavigationBarProps {
  tabNames: string[];
}

const NavigationTabs = (props: NavigationBarProps): React.ReactElement => {
  const { tabNames } = props;
  const classes = useStyles();
  const [value, setValue] = React.useState(0);

  // eslint-disable-next-line @typescript-eslint/ban-types
  const handleChange = (event: React.ChangeEvent<{}>, newValue: number) => {
    setValue(newValue);
  };

  const populateTabs = () => {
    const tabs = tabNames.map((value, index) => <Tab key={index} label={value} />);
    return tabs;
  };

  const populateTabPanel = () => {
    return (
      <TabPanel key={'tab' + value} value={value} index={value}>
        {tabNames[value]}
      </TabPanel>
    );
  };

  return (
    <div className={classes.root}>
      <AppBar position="static">
        <Tabs value={value} onChange={handleChange} aria-label="navigation-tabs">
          {populateTabs()}
        </Tabs>
      </AppBar>
      {populateTabPanel()}
    </div>
  );
};

export default NavigationTabs;
