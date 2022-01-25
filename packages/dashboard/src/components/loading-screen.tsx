import { Backdrop, CircularProgress, Typography, useTheme } from '@mui/material';
import { styled } from '@mui/material';
import { Check as SuccessIcon, ErrorOutline as ErrorIcon } from '@mui/icons-material';
import React from 'react';

export interface LoadingScreenProps extends React.PropsWithChildren<{}> {
  caption?: string;
  variant?: 'loading' | 'success' | 'error';
}

const prefix = 'loading-screen';
const classes = {
  root: `${prefix}-root`,
  caption: `${prefix}-caption`,
};
const StyledDiv = styled('div')(({ theme }) => ({
  [`& .${classes.root}`]: {
    display: 'flex',
    flexFlow: 'column',
    justifyContent: 'center',
    alignItems: 'center',
    zIndex: theme.zIndex.modal,
    height: '100%',
    position: 'absolute',
  },
  [`& .${classes.caption}`]: {
    marginTop: theme.spacing(4),
  },
}));

export default function LoadingScreen(props: LoadingScreenProps): JSX.Element {
  const { caption, children } = props;
  const theme = useTheme();
  return (
    <StyledDiv>
      <Backdrop className={classes.root} open={!!caption}>
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
      {children}
    </StyledDiv>
  );
}
