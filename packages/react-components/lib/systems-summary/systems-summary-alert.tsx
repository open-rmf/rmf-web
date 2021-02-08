import React from 'react';
import { makeStyles, Typography, Button, Badge } from '@material-ui/core';
import NotificationsIcon from '@material-ui/icons/Notifications';
import { NotificationsDialog, Notification } from '../index';

export interface SystemSummaryAlertProps {
  notifications: Notification[];
  deletedNotifications: Notification[];
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

export const SystemSummaryAlert = (props: SystemSummaryAlertProps): JSX.Element => {
  const classes = useStyles();
  const { notifications, deletedNotifications } = props;

  // filter out the deleted notifications
  const filteredRmfNotifications = React.useMemo(() => {
    return notifications.filter((notification) => {
      return !deletedNotifications.some((deletedNot) => notification.id === deletedNot.id);
    });
  }, [notifications, deletedNotifications]);

  const [showNotifications, setShowNotifications] = React.useState(false);
  const getLabel =
    filteredRmfNotifications.length > 0
      ? `${classes.h2} ${classes.hasNotification}`
      : `${classes.h2} ${classes.noNotification}`;

  return (
    <React.Fragment>
      <Typography variant="h6">Notifications</Typography>
      <h2 className={getLabel}>
        <span className={classes.span}>
          {filteredRmfNotifications.length > 0
            ? filteredRmfNotifications.length + ' Alerts'
            : 'No Alerts'}
        </span>
      </h2>
      <Button
        disabled={filteredRmfNotifications.length === 0}
        variant="contained"
        color="primary"
        className={classes.button}
        onClick={() => setShowNotifications(true)}
      >
        Notifications
        <Badge badgeContent={filteredRmfNotifications.length} color="error">
          <NotificationsIcon />
        </Badge>
      </Button>
      <NotificationsDialog
        notifications={filteredRmfNotifications}
        showNotificationsDialog={showNotifications}
        onClose={() => setShowNotifications(false)}
      />
    </React.Fragment>
  );
};
