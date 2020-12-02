import React from 'react';
import { AlertDialog, AlertDialogProps } from './alert-dialog';

export interface ConfirmationAlertDialogProps extends Omit<AlertDialogProps, 'title' | 'variant'> {
  title?: string;
}

/**
 * A specialized `AlertDialog` for confirmation messages.
 */
export const ConfirmationAlertDialog = (props: ConfirmationAlertDialogProps): JSX.Element => {
  const { title, positiveText, negativeText, ...otherProps } = props;
  return (
    <AlertDialog
      title={title || 'Are you sure you want to continue?'}
      positiveText={positiveText || 'OK'}
      negativeText={negativeText || 'Cancel'}
      variant="warn"
      {...otherProps}
    />
  );
};

export default ConfirmationAlertDialog;
