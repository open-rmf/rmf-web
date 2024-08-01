import { styled, Tab, TabProps } from '@mui/material';

const StyledTab = styled((props: TabProps) => <Tab {...props} />)(({ theme }) => ({
  color: theme.palette.primary.contrastText,
  opacity: 0.6,
  '&.Mui-selected': {
    color: theme.palette.primary.contrastText,
    opacity: 1,
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
