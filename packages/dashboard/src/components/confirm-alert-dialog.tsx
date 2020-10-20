import React from 'react';
import Button from '@material-ui/core/Button';
import Dialog from '@material-ui/core/Dialog';
import DialogActions from '@material-ui/core/DialogActions';
import DialogContent from '@material-ui/core/DialogContent';
import DialogContentText from '@material-ui/core/DialogContentText';
import DialogTitle from '@material-ui/core/DialogTitle';
import Slide from '@material-ui/core/Slide';
import { TransitionProps } from '@material-ui/core/transitions';
import WarningIcon from '@material-ui/icons/Warning';
import { makeStyles } from '@material-ui/core';

const Transition = React.forwardRef(function Transition(
  props: TransitionProps & { children?: React.ReactElement<any, any> },
  ref: React.Ref<unknown>,
) {
  return <Slide direction="up" ref={ref} {...props} />;
});

export interface ConfirmAlertDialogProps {
  open: boolean; // controls when the modal should open
  close: () => void; // close the modal
  title?: string;
  content?: string;
  showIcon?: boolean;
  confirmButtonText?: string;
  cancelButtonText?: string;
  confirmCallback?: () => void;
  cancelCallback?: () => void;
}

/**
 * Modal to warn a user about something related to an action.
 */
export const ConfirmAlertDialog = (props: ConfirmAlertDialogProps) => {
  const {
    open,
    close,
    title,
    content,
    showIcon,
    confirmButtonText,
    cancelButtonText,
    confirmCallback,
    cancelCallback,
  } = props;

  const handleCancelButton = () => {
    cancelCallback && cancelCallback();
    close();
  };

  const handleConfirmButton = () => {
    confirmCallback && confirmCallback();
    close();
  };
  const classes = useStyles();

  return (
    <div>
      <Dialog
        open={open}
        TransitionComponent={Transition}
        keepMounted
        onClose={close}
        aria-labelledby="alert-dialog-slide-title"
        aria-describedby="alert-dialog-slide-description"
      >
        {showIcon && (
          <div className={classes.icon}>
            <WarningIcon
              color={'secondary'}
              style={{
                fontSize: '8rem',
              }}
            />
          </div>
        )}

        <DialogTitle id="alert-dialog-slide-title" className={classes.title}>
          {title || 'Are you sure you want to continue?'}
        </DialogTitle>
        <DialogContent>
          <DialogContentText id="alert-dialog-slide-description">
            {content || 'Once you accept this there is no turning back.'}
          </DialogContentText>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleCancelButton} color="primary">
            {cancelButtonText || 'No'}
          </Button>
          <Button onClick={handleConfirmButton} color="secondary">
            {confirmButtonText || `Yes, I'm sure.`}
          </Button>
        </DialogActions>
      </Dialog>
    </div>
  );
};

const useStyles = makeStyles(() => ({
  icon: {
    height: '8rem',
    alignSelf: 'center',
  },
  title: {
    alignSelf: 'center',
  },
}));
