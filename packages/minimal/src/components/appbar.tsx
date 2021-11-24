import React from 'react';
import { Tab, makeStyles } from '@material-ui/core';
import { HeaderBar, LogoButton, NavigationBar, useAsync } from 'react-components';
import { getLogo } from '../utils/simple-icon-manager';

const useStyles = makeStyles((theme) => ({
  appBar: {
    zIndex: theme.zIndex.drawer + 1,
  },
  logoBtn: {
    width: 180,
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
      <NavigationBar>
        <Tab label="action panel" value="action panel" aria-label="Action Panel" />
      </NavigationBar>
    </HeaderBar>
  );
};
