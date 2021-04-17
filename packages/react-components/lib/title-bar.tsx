import React from 'react';
import { createStyles, makeStyles, AppBar, Theme, Toolbar, Typography } from '@material-ui/core';
import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';

export interface TitleBarProps {
  logoPath: string;
  tabNames: string[];
  children?: React.ReactNode;
}

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    root: {
      flexGrow: 1,
      width: '100%',
      alignItems: 'center',
    },
    logo: {
      maxHeight: 30,
      marginTop: theme.spacing(1),
      marginBottom: theme.spacing(1),
    },
    subtitle: {
      textAlign: 'right',
      flexGrow: 1,
    },
    tab: {
      borderRight: '0.25px solid rgba(251, 252, 255, 0.5)',
    },
  }),
);

const TitleBar = (props: TitleBarProps): React.ReactElement => {
  const { logoPath, children, tabNames } = props;
  const classes = useStyles();

  const [value, setValue] = React.useState(0);

  // eslint-disable-next-line @typescript-eslint/ban-types
  const handleChange = (event: React.ChangeEvent<{}>, newValue: number) => {
    setValue(newValue);
  };

  const populateTabs = () => {
    const tabs = tabNames.map((value, index) => (
      <Tab key={index} label={value} value={index} className={classes.tab} />
    ));
    return tabs;
  };

  return (
    <div className={classes.root}>
      <AppBar id="appbar" position="static">
        <Toolbar>
          <img src={logoPath} alt="logo" className={classes.logo} />
          <Tabs
            variant="scrollable"
            scrollButtons="auto"
            value={value}
            onChange={handleChange}
            aria-label="navigation-tabs"
          >
            {populateTabs()}
          </Tabs>
          <Typography variant="caption" className={classes.subtitle}>
            Powered by OpenRMF
          </Typography>
          {children}
        </Toolbar>
      </AppBar>
    </div>
  );
};

export default TitleBar;
