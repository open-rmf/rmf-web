import { CircularProgress, CircularProgressProps, styled } from '@mui/material';
import React from 'react';

const classes = {
  root: 'loading-root',
  container: 'loading-container',
  loadingProgress: 'loading-progress',
  loadingOverlay: 'loading-overlay',
};
const StyledDiv = styled('div')(() => ({
  [`&.${classes.root}`]: {
    position: 'relative',
    height: '100%',
    flex: '1 1 auto',
  },
  [`& .${classes.container}`]: {
    position: 'absolute',
    display: 'flex',
    justifyContent: 'center',
    alignItems: 'center',
    left: 0,
    top: 0,
    width: '100%',
    height: '100%',
  },
  [`& .${classes.loadingProgress}`]: {
    position: 'absolute',
    flex: '0 0 auto',
  },
  [`& .${classes.loadingOverlay}`]: {
    filter: 'blur(2px)',
    opacity: 0.6,
    pointerEvents: 'none',
    userSelect: 'none',
  },
}));

export interface LoadingProps extends React.PropsWithoutRef<CircularProgressProps> {
  children: React.ReactNode;
  loading?: boolean;
  // Hides children when loading is enabled
  hideChildren?: boolean;
  // class added to children container when loading is enabled
  loadingClassName?: string;
}

export function Loading({
  children,
  loading = false,
  hideChildren = false,
  loadingClassName = '',
  className = '',
  style = {},
  ...otherProps
}: LoadingProps): JSX.Element {
  return loading ? (
    <StyledDiv className={classes.root}>
      <div
        className={`${classes.loadingOverlay} ${loadingClassName}`}
        style={{ visibility: hideChildren ? 'hidden' : 'visible' }}
      >
        {children}
      </div>
      <div className={classes.container}>
        <CircularProgress
          aria-label="loading"
          className={`${classes.loadingProgress} ${className}`}
          style={{ visibility: loading ? 'visible' : 'hidden', ...style }}
          {...otherProps}
        />
      </div>
    </StyledDiv>
  ) : (
    <>{children}</>
  );
}
