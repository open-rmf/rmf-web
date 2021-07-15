import { Button, DialogActions, DialogActionsProps } from '@material-ui/core';
import React from 'react';
import { Loading } from './loading';

export interface ConfirmDialogActionsProps extends DialogActionsProps {
  confirmText?: string;
  cancelText?: string;
  loading?: boolean;
  classes?: DialogActionsProps['classes'] & { button: string };
  onConfirmClick?: React.MouseEventHandler;
  onCancelClick?: React.MouseEventHandler;
}

export function ConfirmDialogActions({
  confirmText = 'OK',
  cancelText = 'Cancel',
  loading = false,
  classes,
  onConfirmClick,
  onCancelClick,
  ...otherProps
}: ConfirmDialogActionsProps): JSX.Element {
  return (
    <DialogActions {...otherProps}>
      <Button
        variant="outlined"
        color="secondary"
        aria-label={cancelText}
        onClick={onCancelClick}
        disabled={loading}
        className={classes?.button}
      >
        {cancelText}
      </Button>
      <Button
        variant="contained"
        color="primary"
        aria-label={confirmText}
        disabled={loading}
        onClick={onConfirmClick}
        className={classes?.button}
      >
        <Loading hideChildren loading={loading} size="1.5em" color="inherit">
          {confirmText}
        </Loading>
      </Button>
    </DialogActions>
  );
}
