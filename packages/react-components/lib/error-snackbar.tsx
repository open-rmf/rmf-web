import { Snackbar, SnackbarProps, Alert } from '@mui/material';
import React from 'react';

export interface ErrorSnackbarProps extends SnackbarProps {
  message: string;
}

export function ErrorSnackbar({ message, ...otherProps }: ErrorSnackbarProps): JSX.Element {
  return (
    <Snackbar autoHideDuration={2000} {...otherProps}>
      <Alert severity="error">
        {message.split('\n').length > 0
          ? message.split('\n').map((m, i) => <div key={i}>{m}</div>)
          : message}
      </Alert>
    </Snackbar>
  );
}
