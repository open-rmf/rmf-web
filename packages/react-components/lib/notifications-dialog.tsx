import React from 'react';
import {
  Button,
  Dialog,
  DialogTitle,
  makeStyles,
  Typography,
  Divider,
  DialogActions,
  DialogContent,
  IconButton,
} from '@material-ui/core';
import CloseIcon from '@material-ui/icons/Close';

export interface NotificationDialogProps {
  showNotificationsDialog: boolean;
  setShowNotifications: (payload: boolean) => void;
  notifications: { [key: string]: string }[];
}

const useStyles = makeStyles((theme) => ({
  errorColumn: {
    display: 'flex',
    flexFlow: 'row',
    padding: '0.5rem',
  },
  detailLine: {
    display: 'inline-flex',
    padding: theme.spacing(0.5),
    width: '100%',
  },
  detailLineContent: {
    marginLeft: theme.spacing(0.5),
  },
  closeButton: {
    position: 'absolute',
    right: theme.spacing(1),
    top: theme.spacing(1),
    color: theme.palette.grey[500],
  },
  dialogActions: {
    margin: '0',
    padding: theme.spacing(1),
  },
  dialogContent: {
    padding: theme.spacing(5),
  },
}));

export const NotificationsDialog = (props: NotificationDialogProps): JSX.Element => {
  const classes = useStyles();
  const { showNotificationsDialog, setShowNotifications, notifications } = props;

  return (
    <Dialog
      open={showNotificationsDialog}
      onClose={() => setShowNotifications(false)}
      fullWidth={true}
      maxWidth={'md'}
    >
      <DialogTitle>
        Notifications
        <IconButton
          aria-label="close"
          className={classes.closeButton}
          onClick={() => setShowNotifications(false)}
        >
          <CloseIcon />
        </IconButton>
      </DialogTitle>
      <DialogContent className={classes.dialogContent} dividers>
        {notifications.length > 0 ? (
          notifications.map((notification, i) => {
            return (
              <React.Fragment key={notification.time + '_' + i}>
                <div className={classes.errorColumn}>
                  {Object.keys(notification).map((key) => {
                    return (
                      <div key={key} className={classes.detailLine}>
                        <Typography variant="body1">
                          <b>{key}:</b>
                        </Typography>
                        <Typography variant="body1" className={classes.detailLineContent}>
                          {notification[key]}
                        </Typography>
                      </div>
                    );
                  })}
                </div>
                <Divider />
              </React.Fragment>
            );
          })
        ) : (
          <Typography variant="body1">No Notifications at the moment</Typography>
        )}
      </DialogContent>
      <DialogActions className={classes.dialogActions}>
        <Button autoFocus onClick={() => setShowNotifications(false)} color="primary">
          CLOSE
        </Button>
      </DialogActions>
    </Dialog>
  );
};
