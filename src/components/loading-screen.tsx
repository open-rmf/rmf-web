import { Backdrop, CircularProgress, makeStyles, Typography, useTheme } from '@material-ui/core';
import { Check as SuccessIcon, ErrorOutline as ErrorIcon } from '@material-ui/icons';
import React from 'react';

export interface LoadingScreenProps {
  caption: string;
  variant?: 'loading' | 'success' | 'error';
}

const useStyles = makeStyles(theme => ({
  root: {
    display: 'flex',
    flexFlow: 'column',
    justifyContent: 'center',
    alignItems: 'center',
    zIndex: theme.zIndex.drawer + 1,
  },
  caption: {
    marginTop: theme.spacing(4),
  },
}));

export default function LoadingScreen(props: LoadingScreenProps) {
  const theme = useTheme();
  const classes = useStyles();
  return (
    <Backdrop className={classes.root} open={true}>
      {(props.variant === undefined || props.variant === 'loading') && (
        <CircularProgress size="8rem" />
      )}
      {props.variant === 'success' && (
        <SuccessIcon htmlColor={theme.palette.success.main} style={{ fontSize: '8rem' }} />
      )}
      {props.variant === 'error' && <ErrorIcon color="error" style={{ fontSize: '8rem' }} />}
      <Typography
        className={classes.caption}
        variant="button"
        style={{ color: theme.palette.getContrastText('#000000') }}
      >
        {props.caption}
      </Typography>
    </Backdrop>
  );
}
