import { Button, DialogActions, DialogActionsProps } from '@material-ui/core';
import React from 'react';
import { Loading } from './loading';

export interface ConfirmDialogActionsProps extends DialogActionsProps {
  confirmText?: string;
  cancelText?: string;
  classes?: DialogActionsProps['classes'] & { button: string };
  confirmAction?: () => Promise<void> | void;
  onCancelClick?: React.MouseEventHandler;
}

export function ConfirmDialogActions({
  confirmText = 'OK',
  cancelText = 'Cancel',
  classes,
  confirmAction,
  onCancelClick,
  ...otherProps
}: ConfirmDialogActionsProps): JSX.Element {
  const [confirming, setConfirming] = React.useState(false);

  const handleOkClick = React.useCallback(() => {
    if (!confirmAction) {
      return;
    }
    setConfirming(true);
    (async () => {
      await confirmAction();
      setConfirming(false);
    })();
  }, [confirmAction]);

  return (
    <DialogActions {...otherProps}>
      <Button
        variant="outlined"
        color="secondary"
        aria-label={cancelText}
        onClick={onCancelClick}
        disabled={confirming}
        className={classes?.button}
      >
        {cancelText}
      </Button>
      <Button
        variant="contained"
        color="primary"
        aria-label={confirmText}
        disabled={confirming}
        onClick={handleOkClick}
        className={classes?.button}
      >
        <Loading hideChildren loading={confirming} size="1.5em" color="inherit">
          {confirmText}
        </Loading>
      </Button>
    </DialogActions>
  );
}
