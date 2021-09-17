import { styled } from '@material-ui/core';
import Tabs, { TabsProps } from '@material-ui/core/Tabs';
import React from 'react';

const classes = {
  tabsContainer: 'navigation-bar-root',
};
const NavigationBarRoot = styled((props: TabsProps) => <Tabs {...props} />)(() => ({
  [`&.${classes.tabsContainer}`]: {
    borderRight: '0.25px solid rgba(251, 252, 255, 0.5)',
    borderLeft: '0.25px solid rgba(251, 252, 255, 0.5)',
    flexGrow: 4,
  },
}));

export interface NavigationBarProps {
  value?: string;
  onTabChange?(event: React.ChangeEvent<unknown>, newValue: unknown): void;
  children?: React.ReactNode;
}

export const NavigationBar = (props: NavigationBarProps): JSX.Element => {
  const { value, onTabChange, children } = props;
  return (
    <NavigationBarRoot
      variant="scrollable"
      scrollButtons="auto"
      value={value}
      onChange={onTabChange}
      className={classes.tabsContainer}
      TabIndicatorProps={{ style: { backgroundColor: '#d32f2f' } }}
    >
      {children}
    </NavigationBarRoot>
  );
};
