import React from 'react';
import { makeStyles } from '@material-ui/core';
import ErrorIcon from '@material-ui/icons/Error';
import { Typography, Grid } from '@material-ui/core';

const useStyles = makeStyles((theme) => ({
  errorIcon: {
    color: theme.palette.error.main,
  },
  errorMsg: {
    margin: '0 0.5rem',
  },
  errorDisabled: {
    pointerEvents: 'none',
    filter: 'blur(.25rem)',
    gridArea: '1 / 1',
  },
  overlay: {
    gridArea: '1 / 1',
    marginTop: '1rem',
    backdropFilter: 'blur(.5rem)',
    padding: theme.spacing(1),
  },
  container: {
    display: 'grid',
  },
  disableSelect: {
    userSelect: 'none',
  },
}));

export interface ErrorOverlayProps {
  errorMsg?: string;
  children: JSX.Element | null;
  overrideErrorStyle?: string;
}

export const ErrorOverlay = (props: ErrorOverlayProps): JSX.Element => {
  const classes = useStyles();
  const { errorMsg, children, overrideErrorStyle } = props;

  return errorMsg ? (
    <React.Fragment>
      <div className={classes.container}>
        <div className={classes.errorDisabled}>{children}</div>
        <div
          className={
            children ? `${classes.overlay} ${classes.disableSelect}` : classes.disableSelect
          }
        >
          <Grid container direction="row" justify="center" alignItems="center" spacing={2}>
            <Grid item>
              <Typography color="error" variant="h6" align="center">
                Error
              </Typography>
            </Grid>
            <Grid item>
              <ErrorIcon className={classes.errorIcon} />
            </Grid>
          </Grid>
          <Typography
            className={overrideErrorStyle ? overrideErrorStyle : classes.errorMsg}
            color="error"
            variant="h6"
            align="center"
          >
            {errorMsg ? errorMsg : 'Unknown error'}
          </Typography>
        </div>
      </div>
    </React.Fragment>
  ) : (
    <React.Fragment>{children}</React.Fragment>
  );
};
