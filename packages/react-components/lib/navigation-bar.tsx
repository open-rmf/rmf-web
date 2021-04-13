import React from 'react';
import { makeStyles, Theme } from '@material-ui/core/styles';
import AppBar from '@material-ui/core/AppBar';
import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';

interface NavigationBarProps {
  tabNames: string[];
}
const useStyles = makeStyles((theme: Theme) => ({
  root: {
    flexGrow: 1,
    backgroundColor: theme.palette.background.paper,
  },
}));

const NavigationBar = (props: NavigationBarProps): React.ReactElement => {
  const classes = useStyles();
  const { tabNames } = props;
  const [value, setValue] = React.useState(0);

  // eslint-disable-next-line @typescript-eslint/ban-types
  const handleChange = (event: React.ChangeEvent<{}>, newValue: number) => {
    setValue(newValue);
  };

  const populateTabs = () => {
    const tabs = tabNames.map((value, index) => (
      <Tab key={index} label={value} value={tabNames[index]} />
    ));
    return tabs;
  };

  return (
    <div className={classes.root}>
      <AppBar position="static">
        <Tabs value={value} onChange={handleChange} aria-label="navigation-tabs">
          {populateTabs()}
        </Tabs>
      </AppBar>
    </div>
  );
};

export default NavigationBar;
