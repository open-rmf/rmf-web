import React from 'react';
import { createStyles, makeStyles } from '@material-ui/core';
import Tabs from '@material-ui/core/Tabs';

const useStyles = makeStyles(() =>
  createStyles({
    tabsContainer: {
      borderRight: '0.25px solid rgba(251, 252, 255, 0.5)',
      borderLeft: '0.25px solid rgba(251, 252, 255, 0.5)',
      flexGrow: 4,
    },
  }),
);

interface NavigationBarProps {
  value?: string;
  onTabChange?: (event: React.ChangeEvent<unknown>, newValue: string) => void;
  children?: React.ReactNode;
}

export const NavigationBar = (props: NavigationBarProps): JSX.Element => {
  const { value, onTabChange, children } = props;
  const classes = useStyles();

  return (
    <Tabs
      variant="scrollable"
      scrollButtons="auto"
      value={value}
      onChange={onTabChange}
      className={classes.tabsContainer}
    >
      {children}
    </Tabs>
  );
};
