import React from 'react';
import {
  Button,
  Dialog,
  DialogTitle,
  makeStyles,
  Typography,
  DialogActions,
  DialogContent,
  IconButton,
  Paper,
  Select,
} from '@material-ui/core';
import CloseIcon from '@material-ui/icons/Close';
import FilterList from '@material-ui/icons/FilterList';

export interface NotificationDialogProps {
  showNotificationsDialog: boolean;
  setShowNotifications: (payload: boolean) => void;
  notifications: { [key: string]: string }[];
}

const useStyles = makeStyles((theme) => ({
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
  paper: {
    display: 'inline-flex',
    justifyContent: 'space-between',
    padding: theme.spacing(1),
    width: '100%',
    margin: `0.5rem 0`,
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
      maxWidth={'sm'}
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
        <Select IconComponent={() => <FilterList />}></Select>
        {notifications.length > 0 ? (
          notifications.map((notification, i) => {
            return (
              <React.Fragment key={notification.time + '_' + i}>
                <Paper elevation={3} className={classes.paper}>
                  <Typography variant="body1">{notification.time}:</Typography>
                  <Typography variant="body1">{notification.error}</Typography>
                </Paper>
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
