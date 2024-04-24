import { styled } from '@mui/material';
import Tabs, { TabsProps } from '@mui/material/Tabs';
import React from 'react';

const classes = {
  tabsContainer: 'navigation-bar-root',
};
const StyledTabs = styled((props: TabsProps) => <Tabs {...props} />)(() => ({
  [`&.${classes.tabsContainer}`]: {
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
    <StyledTabs
      variant="scrollable"
      scrollButtons="auto"
      value={value || false}
      onChange={onTabChange}
      className={classes.tabsContainer}
      TabIndicatorProps={{ style: { backgroundColor: '#d32f2f' } }}
    >
      {children}
    </StyledTabs>
  );
};
