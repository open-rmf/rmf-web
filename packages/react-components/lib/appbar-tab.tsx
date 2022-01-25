import { Tab, TabProps, styled } from '@mui/material';
import React from 'react';

const StyledTab = styled((props: TabProps) => <Tab {...props} />)(({ theme }) => ({
  color: 'rgba(255, 255, 255, 0.7)',
  '&.Mui-selected': {
    color: theme.palette.text.primary,
  },
  '&.Mui-focusVisible': {
    backgroundColor: 'rgba(100, 95, 228, 0.32)',
  },
}));

export interface TabPanelProps extends TabProps {
  label: string;
  value: string;
  onTabClick?: () => void;
}

export function AppBarTab(props: TabPanelProps): JSX.Element {
  const { label, value, onTabClick, ...otherProps } = props;

  return (
    <StyledTab
      label={label}
      value={value}
      onClick={() => onTabClick && onTabClick()}
      {...otherProps}
    />
  );
}
