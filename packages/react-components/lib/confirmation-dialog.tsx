import {
  Button,
  Dialog,
  DialogActions,
  DialogActionsProps,
  DialogContent,
  DialogProps,
  DialogTitle,
  Grid,
  styled,
} from '@mui/material';
import clsx from 'clsx';
import React from 'react';
import { Loading } from './loading';

const dialogClasses = {
  title: 'confirmation-dialogue-info-value',
  actionBtn: 'confirmation-dialogue-action-button',
};
const StyledDialog = styled((props: DialogProps) => <Dialog {...props} />)(() => ({
  [`& .${dialogClasses.title}`]: {
    flex: '1 1 auto',
  },
  [`& .${dialogClasses.actionBtn}`]: {
    minWidth: 80,
  },
}));

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
  return (
    <StyledDialog onClose={onClose} {...otherProps}>
      <form
        onSubmit={(ev) => {
          ev.preventDefault();
          onSubmit && onSubmit(ev);
        }}
        aria-label={title}
      >
        <DialogTitle>
          <Grid container wrap="nowrap">
            <Grid item className={dialogClasses.title}>
              {title}
            </Grid>
            <Grid item>{toolbar}</Grid>
          </Grid>
        </DialogTitle>
        <DialogContent>{children}</DialogContent>
        <DialogActions>
          <Button
            variant="outlined"
            onClick={(ev) => onClose && onClose(ev, 'escapeKeyDown')}
            disabled={submitting}
            className={clsx(dialogClasses.actionBtn, classes?.button)}
          >
            {cancelText}
          </Button>
          <Button
            variant="contained"
            type="submit"
            color="primary"
            disabled={submitting}
            className={clsx(dialogClasses.actionBtn, classes?.button)}
          >
            <Loading hideChildren loading={submitting} size="1.5em" color="inherit">
              {confirmText}
            </Loading>
          </Button>
        </DialogActions>
      </form>
    </StyledDialog>
  );
}
