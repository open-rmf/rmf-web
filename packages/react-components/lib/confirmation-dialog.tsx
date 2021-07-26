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
import clsx from 'clsx';
import React from 'react';
import { Loading } from './loading';

const useStyles = makeStyles({
  title: {
    flex: '1 1 auto',
  },
  actionBtn: {
    minWidth: 80,
  },
});

export interface ConfirmationDialogProps extends DialogProps {
  title?: string;
  confirmText?: string;
  cancelText?: string;
  // disable the dialog actions and shows a loading indicator
  submitting?: boolean;
  classes?: DialogActionsProps['classes'] & { button: string };
  toolbar?: React.ReactNode;
  onSubmit?: React.FormEventHandler;
}

export function ConfirmationDialog({
  title = 'Confirm',
  confirmText = 'OK',
  cancelText = 'Cancel',
  submitting = false,
  classes,
  onSubmit,
  toolbar,
  onClose,
  children,
  ...otherProps
}: ConfirmationDialogProps): JSX.Element {
  const myClasses = useStyles();
  return (
    <Dialog onClose={onClose} {...otherProps}>
      <form
        onSubmit={(ev) => {
          ev.preventDefault();
          onSubmit && onSubmit(ev);
        }}
        aria-label="form"
      >
        <DialogTitle>
          <Grid container wrap="nowrap">
            <Grid item className={myClasses.title}>
              {title}
            </Grid>
            <Grid item>{toolbar}</Grid>
          </Grid>
        </DialogTitle>
        <DialogContent>{children}</DialogContent>
        <DialogActions>
          <Button
            variant="outlined"
            color="secondary"
            aria-label={cancelText}
            onClick={(ev) => onClose && onClose(ev, 'escapeKeyDown')}
            disabled={submitting}
            className={clsx(myClasses.actionBtn, classes?.button)}
          >
            {cancelText}
          </Button>
          <Button
            variant="contained"
            type="submit"
            color="primary"
            aria-label={confirmText}
            disabled={submitting}
            className={clsx(myClasses.actionBtn, classes?.button)}
          >
            <Loading hideChildren loading={submitting} size="1.5em" color="inherit">
              {confirmText}
            </Loading>
          </Button>
        </DialogActions>
      </form>
    </Dialog>
  );
}
