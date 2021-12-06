import React from 'react';
import ErrorIcon from '@mui/icons-material/Error';
import { Typography, Grid, styled } from '@mui/material';

const classes = {
  errorIcon: 'erroroverlay-error-icon',
  errorMsg: 'erroroverlay-error-msg',
  errorDisabled: 'erroroverlay-error-disabled',
  overlay: 'erroroverlay-overlay',
  container: 'erroroverlay-container',
  disableSelect: 'erroroverlay-disable-select',
};
const StyledDiv = styled('div')(({ theme }) => ({
  [`& .${classes.errorIcon}`]: {
    color: theme.palette.error.main,
    fontSize: '2rem',
  },
  [`& .${classes.errorMsg}`]: {
    margin: '0.5rem',
  },
  [`& .${classes.errorDisabled}`]: {
    pointerEvents: 'none',
    filter: 'blur(.25rem)',
    gridArea: '1 / 1',
    opacity: 0.6,
  },
  [`& .${classes.overlay}`]: {
    gridArea: '1 / 1',
    backdropFilter: 'blur(.5rem)',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
  },
  [`&.${classes.container}`]: {
    display: 'grid',
  },
  [`& .${classes.disableSelect}`]: {
    userSelect: 'none',
  },
}));

export interface ErrorOverlayProps {
  errorMsg?: string | null;
  children: React.ReactNode | null;
  overrideErrorStyle?: string;
}

export const ErrorOverlay = React.memo((props: ErrorOverlayProps): JSX.Element => {
  const { errorMsg, children, overrideErrorStyle } = props;

  return errorMsg ? (
    <StyledDiv className={classes.container}>
      <div className={classes.errorDisabled}>{children}</div>
      <div
        className={children ? `${classes.overlay} ${classes.disableSelect}` : classes.disableSelect}
      >
        <div>
          <Grid container direction="row" justifyContent="center" alignItems="center" spacing={2}>
            <Grid item>
              <ErrorIcon className={classes.errorIcon} />
            </Grid>
            <Grid item>
              <Typography color="error" variant="h4" align="center">
                Error
              </Typography>
            </Grid>
          </Grid>
          <Typography
            className={overrideErrorStyle ? overrideErrorStyle : classes.errorMsg}
            color="error"
            variant="h6"
            align="center"
          >
            {errorMsg}
          </Typography>
        </div>
      </div>
    </StyledDiv>
  ) : (
    <React.Fragment>{children}</React.Fragment>
  );
});
