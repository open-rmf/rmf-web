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
  MenuItem,
  Input,
} from '@material-ui/core';
import CloseIcon from '@material-ui/icons/Close';
import RestoreIcon from '@material-ui/icons/Restore';
import CheckIcon from '@material-ui/icons/Check';

export interface Notification {
  time: string;
  error: string;
  severity: string;
}

export interface NotificationDialogProps {
  showNotificationsDialog: boolean;
  setShowNotifications: (payload: boolean) => void;
  notifications: Notification[];
}

interface SelectChangeEvent {
  name?: string | undefined;
  value: unknown;
}

interface SeverityIndicatoryProps {
  severity: string;
  className?: string;
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
    display: 'grid',
    gridTemplateColumns: '1fr repeat(2, 3fr) 1fr',
    textAlign: 'center',
    padding: theme.spacing(1),
    width: '100%',
    margin: `0.5rem 0`,
  },
  filter: {
    display: 'flex',
    float: 'right',
    marginBottom: theme.spacing(1),
  },
  select: {
    width: '150px',
    padding: '0 0.5rem',
  },
  checkIcon: {
    color: theme.palette.success.main,
    padding: '0',
  },
  legend: {
    display: 'flex',
  },
  indicator: {
    padding: '0',
    fontWeight: 600,
  },
  placeholder: {
    color: theme.palette.success.main,
  },
}));

const severityStyles = makeStyles((theme) => ({
  high: {
    color: theme.palette.secondary.dark,
  },
  medium: {
    color: theme.palette.error.main,
  },
  low: {
    color: theme.palette.warning.light,
  },
}));

const SeverityIndicator = (props: SeverityIndicatoryProps): JSX.Element => {
  const { severity, className } = props;
  const classes = severityStyles();

  const getStatusLabelClass = (severity: string): string => {
    switch (severity) {
      case 'High':
        return classes.high;
      case 'Medium':
        return classes.medium;
      case 'Low':
        return classes.low;
      default:
        return '';
    }
  };

  const styles = `${getStatusLabelClass(severity)} ${className}`;
  return (
    <Typography variant="body1" className={styles} align="left">
      {severity}
    </Typography>
  );
};

export const NotificationsDialog = (props: NotificationDialogProps): JSX.Element => {
  const classes = useStyles();

  const { showNotificationsDialog, setShowNotifications, notifications } = props;

  const [level, setLevel] = React.useState('');
  const [rmfNotifications, setRmfNotifications] = React.useState(notifications);
  // a copy of rmfNotifications
  // ensure notifications can be restored to correct state after filtering
  const [notficationsCopy, setNotificationsCopy] = React.useState(notifications);
  // list of alert level for filtering
  const alertLevel = React.useMemo(() => {
    const holder: string[] = [];
    notifications.forEach((notification) => {
      if (!holder.includes(notification.severity)) {
        holder.push(notification.severity);
      }
    });
    return holder;
  }, [notifications]);

  // handle filter change
  const handleChange = (e: React.ChangeEvent<SelectChangeEvent>) => {
    const val = e.target.value as string;
    setLevel(val);
    const filterNotifications: Notification[] = [];

    notficationsCopy.forEach((notification) => {
      if (notification.severity === val) {
        filterNotifications.push(notification);
      }
    });
    setRmfNotifications(filterNotifications);
  };

  // restore filtered notifications
  const restoreNotifications = () => {
    setRmfNotifications(notficationsCopy);
  };

  // delete notifications once marked read
  const deleteReadNotifications = (i: number) => {
    const beforeIndex = rmfNotifications.slice(0, i);
    const afterIndex = rmfNotifications.slice(i + 1);
    const newNotifications = beforeIndex.concat(afterIndex);
    setRmfNotifications(newNotifications);
    // update notificationsCopy to ensure consistent state with rmfNotifications
    setNotificationsCopy(newNotifications);
  };

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
        {rmfNotifications.length > 0 ? (
          <React.Fragment>
            <div className={classes.filter}>
              <Select
                className={classes.select}
                displayEmpty={true}
                value={level}
                onChange={(e) => handleChange(e)}
                input={<Input />}
                renderValue={() => (level === '' ? <em>Filter by severity</em> : level)}
              >
                {alertLevel.map((level) => {
                  return (
                    <MenuItem key={level} value={level}>
                      {level}
                    </MenuItem>
                  );
                })}
              </Select>
              <IconButton onClick={restoreNotifications}>
                <RestoreIcon />
              </IconButton>
            </div>

            <Paper className={classes.paper}>
              <Typography variant="h6" align="left">
                Severity
              </Typography>
              <Typography variant="h6" align="left">
                Date
              </Typography>
              <Typography variant="h6" align="left">
                Message
              </Typography>
              <Typography variant="h6" align="right">
                Resolved
              </Typography>
            </Paper>
            {rmfNotifications.map((notification, i) => {
              return (
                <React.Fragment key={notification.time + '_' + i}>
                  <Paper elevation={3} className={classes.paper}>
                    <SeverityIndicator
                      className={classes.indicator}
                      severity={notification.severity}
                    />
                    <Typography variant="body1" align="left">
                      {notification.time}
                    </Typography>
                    <Typography variant="body1" align="left">
                      {notification.error}
                    </Typography>
                    <Typography align="right">
                      <IconButton
                        className={classes.checkIcon}
                        onClick={() => deleteReadNotifications(i)}
                      >
                        <CheckIcon />
                      </IconButton>
                    </Typography>
                  </Paper>
                </React.Fragment>
              );
            })}
          </React.Fragment>
        ) : (
          <Typography className={classes.placeholder} align="center" variant="h6">
            All Systems Green
          </Typography>
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
