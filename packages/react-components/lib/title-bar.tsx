import { createStyles, makeStyles, AppBar, Theme, Toolbar, Typography } from '@material-ui/core';
import React from 'react';

export interface TitleBarProps {
  logoPath: string;
  children?: React.ReactNode;
}

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    root: {
      flexGrow: 1,
    },
    logo: {
      maxHeight: 30,
      marginTop: theme.spacing(1),
      marginBottom: theme.spacing(1),
    },
    subtitle: {
      textAlign: 'right',
      flexGrow: 1,
    },
  }),
);

const TitleBar = (props: TitleBarProps): React.ReactElement => {
  const { logoPath, children } = props;
  const classes = useStyles();

  return (
    <div className={classes.root}>
      <AppBar id="appbar" position="static">
        <Toolbar>
          <img src={logoPath} alt="logo" className={classes.logo} />
          <Typography variant="caption" className={classes.subtitle}>
            Powered by OpenRMF
          </Typography>
          {children}
        </Toolbar>
      </AppBar>
    </div>
  );
};

export default TitleBar;
