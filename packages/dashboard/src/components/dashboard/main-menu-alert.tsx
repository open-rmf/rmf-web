import React from 'react';
import { makeStyles, Typography, Button, Badge } from '@material-ui/core';
import NotificationsIcon from '@material-ui/icons/Notifications';
import { NotificationsDialog, Notification } from 'react-components';

export interface MainMenuAlertProps {
  notifications: Notification[];
}

const useStyles = makeStyles((theme) => ({
  h2: {
    width: '100%',
    textAlign: 'center',
    borderBottom: '1px solid rgba(0, 0, 0, 0.12)',
    lineHeight: '0.1em',
    margin: '10px 0 20px',
    fontFamily: 'Roboto, Helvetica, Arial, sans-serif',
    fontSize: '1rem',
    fontWeight: 500,
  },
  span: {
    background: 'white',
    padding: '0 10px',
  },
  button: {
    width: '100%',
    margin: '0.5rem 0',
  },
  noNotification: {
    color: theme.palette.success.main,
  },
  hasNotification: {
    color: theme.palette.warning.main,
  },
}));

export const MainMenuAlert = (props: MainMenuAlertProps) => {
  const classes = useStyles();
  const { notifications } = props;

  const [showNotifications, setShowNotifications] = React.useState(false);
  const getLabel =
    notifications.length > 0
      ? `${classes.h2} ${classes.hasNotification}`
      : `${classes.h2} ${classes.noNotification}`;

  return (
    <React.Fragment>
      <Typography variant="h6">Notifications</Typography>
      <h2 className={getLabel}>
        <span className={classes.span}>
          {notifications.length > 0 ? notifications.length + ' Alerts' : 'No Alerts'}
        </span>
      </h2>
      <Button
        disabled={notifications.length === 0}
        variant="contained"
        color="primary"
        className={classes.button}
        onClick={() => setShowNotifications(true)}
      >
        Notifications
        <Badge badgeContent={notifications.length} color="error">
          <NotificationsIcon />
        </Badge>
      </Button>
      <NotificationsDialog
        notifications={notifications}
        showNotificationsDialog={showNotifications}
        setShowNotifications={() => setShowNotifications(false)}
      />
    </React.Fragment>
  );
};
