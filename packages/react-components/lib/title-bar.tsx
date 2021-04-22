import React from 'react';
import { createStyles, makeStyles, AppBar, Theme, Toolbar, Typography } from '@material-ui/core';
import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';
import Badge from '@material-ui/core/Badge';

export interface TitleBarProps {
  logoPath: string;
  tabNames: string[];
  children?: React.ReactNode;
}

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    logo: {
      maxWidth: 120,
      marginLeft: theme.spacing(2),
      marginRight: theme.spacing(2),
      opacity: 1,
    },
    subtitle: {
      textAlign: 'right',
      flexGrow: 1,
    },
    tab: {
      border: '0.25px solid rgba(251, 252, 255, 0.5)',
    },
    toolbar: {
      flexGrow: 1,
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
    <AppBar id="appbar" position="static">
      <Tabs
        variant="scrollable"
        scrollButtons="auto"
        value={value}
        onChange={handleChange}
        aria-label="navigation-tabs"
      >
        <Tab
          label={
            <Badge badgeContent={<img src={logoPath} alt="logo" className={classes.logo} />} />
          }
        />
        {populateTabs()}
        <Toolbar className={classes.toolbar}>
          <Typography variant="caption" className={classes.subtitle}>
            Powered by OpenRMF
          </Typography>
          {children}
        </Toolbar>
      </Tabs>
    </AppBar>
  );
};

export default TitleBar;
