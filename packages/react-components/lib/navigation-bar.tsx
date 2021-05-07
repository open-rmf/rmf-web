import { createStyles, makeStyles } from '@material-ui/core';
import Tabs from '@material-ui/core/Tabs';
import React from 'react';

const useStyles = makeStyles((theme) =>
  createStyles({
    tabsContainer: {
      borderRight: '0.25px solid rgba(251, 252, 255, 0.5)',
      borderLeft: '0.25px solid rgba(251, 252, 255, 0.5)',
      flexGrow: 4,
    },
    indicator: {
      backgroundColor: theme.palette.success.main,
    },
  }),
);

export interface NavigationBarProps {
  value?: string;
  onTabChange?(event: React.ChangeEvent<unknown>, newValue: unknown): void;
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
      classes={{ indicator: classes.indicator }}
    >
      {children}
    </Tabs>
  );
};
