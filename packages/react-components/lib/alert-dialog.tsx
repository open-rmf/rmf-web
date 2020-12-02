import { Button, Dialog, DialogContentText, DialogProps, Slide } from '@material-ui/core';
import MuiDialogActions from '@material-ui/core/DialogActions';
import MuiDialogContent from '@material-ui/core/DialogContent';
import MuiDialogTitle, {
  DialogTitleProps as MuiDialogTitleProps,
} from '@material-ui/core/DialogTitle';
import IconButton from '@material-ui/core/IconButton';
import { makeStyles, Theme, withStyles } from '@material-ui/core/styles';
import { TransitionProps } from '@material-ui/core/transitions/transition';
import Typography from '@material-ui/core/Typography';
import CloseIcon from '@material-ui/icons/Close';
import WarningIcon from '@material-ui/icons/Warning';
import React from 'react';

const useStyles = makeStyles((theme) => ({
  root: {
    margin: 0,
    padding: theme.spacing(2),
  },
  closeButton: {
    position: 'absolute',
    right: theme.spacing(1),
    top: theme.spacing(1),
    color: theme.palette.grey[500],
  },
  icon: {
    height: '8rem',
    alignSelf: 'center',
  },
  title: {
    alignSelf: 'center',
  },
}));

interface DialogTitleProps extends MuiDialogTitleProps {
  children: React.ReactNode;
  onCloseClick?: (ev: React.MouseEvent) => void;
}

const DialogTitle = (props: DialogTitleProps) => {
  const { onCloseClick, children, ...otherProps } = props;
  const classes = useStyles();

  return (
    <MuiDialogTitle disableTypography className={classes.root} {...otherProps}>
      <Typography variant="h6">{children}</Typography>
      {onCloseClick ? (
        <IconButton aria-label="close" className={classes.closeButton} onClick={onCloseClick}>
          <CloseIcon />
        </IconButton>
      ) : null}
    </MuiDialogTitle>
  );
};

const DialogContent = withStyles((theme: Theme) => ({
  root: {
    padding: theme.spacing(2),
  },
}))(MuiDialogContent);

const DialogActions = withStyles((theme: Theme) => ({
  root: {
    margin: 0,
    padding: theme.spacing(1),
  },
}))(MuiDialogActions);

const Transition = React.forwardRef(function Transition(
  props: TransitionProps & { children?: JSX.Element },
  ref: React.Ref<unknown>,
) {
  return <Slide direction="up" ref={ref} {...props} />;
});

export interface AlertDialogProps extends DialogProps {
  title: string;
  variant: 'warn' | 'noIcon';
  message?: string;
  /**
   * default: 'OK'
   */
  positiveText?: string;
  /**
   * The text for the negative button, if not provided, the negative button will not be shown.
   */
  negativeText?: string;
  onPositiveClick?: (ev: React.MouseEvent) => void;
  onNegativeClick?: (ev: React.MouseEvent) => void;
  /**
   * If provided, will show a close button on the dialog.
   */
  onCloseClick?: (ev: React.MouseEvent) => void;
}

/**
 * Modal to warn a user about something related to an action.
 */
export const AlertDialog = React.forwardRef(
  (props: AlertDialogProps, ref: React.Ref<unknown>): JSX.Element => {
    const {
      title,
      variant,
      message,
      positiveText,
      negativeText,
      onPositiveClick,
      onNegativeClick,
      onCloseClick,
      ...otherProps
    } = props;

    const classes = useStyles();

    return (
      <div>
        <Dialog
          ref={ref}
          TransitionComponent={Transition}
          aria-labelledby="alert-dialog-slide-title"
          aria-describedby="alert-dialog-slide-description"
          {...otherProps}
        >
          {variant !== 'noIcon' && (
            <div className={classes.icon}>
              {variant === 'warn' && (
                <WarningIcon
                  color={'secondary'}
                  style={{
                    fontSize: '8rem',
                  }}
                />
              )}
            </div>
          )}

          <DialogTitle
            id="alert-dialog-slide-title"
            onCloseClick={onCloseClick}
            className={classes.title}
          >
            {title}
          </DialogTitle>
          <DialogContent id="alert-dialog-slide-description">
            {message && <DialogContentText>{message}</DialogContentText>}
          </DialogContent>
          <DialogActions>
            {negativeText && (
              <Button onClick={onNegativeClick} color="primary" id="alert-dialog-cancel-button">
                {negativeText}
              </Button>
            )}
            <Button onClick={onPositiveClick} color="secondary" id="alert-dialog-confirm-button">
              {positiveText || 'OK'}
            </Button>
          </DialogActions>
        </Dialog>
      </div>
    );
  },
);

export default AlertDialog;
