import React from 'react';
import { makeStyles, Toolbar, Typography } from '@material-ui/core';
import { HeaderBar, LogoButton, useAsync } from 'react-components';
import { getLogo } from '../utils/simple-icon-manager';

const useStyles = makeStyles((theme) => ({
  appBar: {
    zIndex: theme.zIndex.drawer + 1,
  },
  logoBtn: {
    width: 180,
  },
  toolbar: {
    textAlign: 'right',
    flexGrow: -1,
  },
}));

export const AppBar = () => {
  const classes = useStyles();
  const [logo, setLogo] = React.useState('');
  const safeAsync = useAsync();

  React.useEffect(() => {
    (async () => {
      setLogo(await safeAsync(getLogo()));
    })();
  });

  return (
    <HeaderBar className={classes.appBar}>
      <LogoButton src={logo} alt="logo" className={classes.logoBtn} />
      <Toolbar variant="dense" className={classes.toolbar}>
        <Typography>Minimal App</Typography>
      </Toolbar>
    </HeaderBar>
  );
};
