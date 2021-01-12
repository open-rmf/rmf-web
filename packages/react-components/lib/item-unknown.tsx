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
  },
  container: {
    display: 'grid',
  },
  disableSelect: {
    userSelect: 'none',
  },
}));

export interface ItemUnknownProps {
  errorMsg?: string;
  showError: boolean;
  children: JSX.Element | null;
}

export const ItemUnknown = (props: ItemUnknownProps): JSX.Element => {
  const classes = useStyles();
  const { errorMsg, showError, children } = props;

  return showError ? (
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
          <Typography className={classes.errorMsg} color="error" variant="body1" align="center">
            {errorMsg ? errorMsg : 'Unknown error'}
          </Typography>
        </div>
      </div>
    </React.Fragment>
  ) : (
    <React.Fragment>{children}</React.Fragment>
  );
};
