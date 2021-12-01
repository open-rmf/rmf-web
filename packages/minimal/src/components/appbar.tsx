import React from 'react';
import { Toolbar, Typography, styled } from '@mui/material';
import { HeaderBar, HeaderBarProps, LogoButton, useAsync } from 'react-components';
import { getLogo } from '../utils/simple-icon-manager';

const prefix = 'app-bar';
const classes = {
  appBar: `${prefix}-root`,
  logoBtn: `${prefix}-logo-button`,
  toolbar: `${prefix}-tool-bar`,
};
const StyledHeaderBar = styled((props: HeaderBarProps) => <HeaderBar {...props} />)(
  ({ theme }) => ({
    [`&.${classes.appBar}`]: {
      zIndex: theme.zIndex.drawer + 1,
    },
    [`& .${classes.logoBtn}`]: {
      width: 180,
    },
    [`& .${classes.toolbar}`]: {
      textAlign: 'right',
      flexGrow: -1,
    },
  }),
);

export const AppBar = () => {
  const [logo, setLogo] = React.useState('');
  const safeAsync = useAsync();

  React.useEffect(() => {
    (async () => {
      setLogo(await safeAsync(getLogo()));
    })();
  });

  return (
    <StyledHeaderBar className={classes.appBar}>
      <LogoButton src={logo} alt="logo" className={classes.logoBtn} />
      <Toolbar variant="dense" className={classes.toolbar}>
        <Typography>Minimal App</Typography>
      </Toolbar>
    </StyledHeaderBar>
  );
};
