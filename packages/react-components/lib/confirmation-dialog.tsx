import {
  Button,
  Dialog,
  DialogActions,
  DialogActionsProps,
  DialogContent,
  DialogProps,
  DialogTitle,
  Grid,
  makeStyles,
} from '@material-ui/core';
import React from 'react';
import { Loading } from './loading';

const useStyles = makeStyles({
  title: {
    flex: '1 1 100%',
  },
});

export interface ConfirmationDialogProps extends DialogProps {
  title?: string;
  confirmText?: string;
  cancelText?: string;
  // disable the dialog actions and shows a loading indicator
  loading?: boolean;
  classes?: DialogActionsProps['classes'] & { button: string };
  toolbar?: React.ReactNode;
  onConfirmClick?: React.MouseEventHandler;
  onCancelClick?: React.MouseEventHandler;
}

export function ConfirmationDialog({
  title = 'Confirm',
  confirmText = 'OK',
  cancelText = 'Cancel',
  loading = false,
  classes,
  onConfirmClick,
  onCancelClick,
  toolbar,
  children,
  ...otherProps
}: ConfirmationDialogProps): JSX.Element {
  const myClasses = useStyles();
  return (
    <Dialog {...otherProps}>
      <DialogTitle>
        <Grid container wrap="nowrap">
          <Grid item className={myClasses.title}>
            {title}
          </Grid>
          <Grid item>{toolbar}</Grid>
        </Grid>
      </DialogTitle>
      <DialogContent>{children}</DialogContent>
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
    </Dialog>
  );
}
