import { createStyles, makeStyles } from '@material-ui/core';
import Tabs from '@material-ui/core/Tabs';
import React from 'react';

const useStyles = makeStyles((theme) =>
  createStyles({
    tabsContainer: {
      flexGrow: 4,
      borderRight: `0.25px solid ${theme.palette.text.primary}`,
      borderLeft: `0.25px solid ${theme.palette.text.primary}`,
    },
  }),
);

export interface NavigationBarProps {
  value?: string;
  onTabChange?(event: React.ChangeEvent<unknown>, newValue: unknown): void;
  children?: React.ReactNode;
  navBarTheme?: string;
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
