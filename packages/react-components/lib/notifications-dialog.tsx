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
import moment from 'moment';

export interface Notification {
  id: number;
  time: string;
  error: string;
  severity: Severity;
}

export interface NotificationDialogProps {
  showNotificationsDialog: boolean;
  onClose: () => void;
  notifications: Notification[];
}

export enum Severity {
  Low = 'Low',
  Medium = 'Medium',
  High = 'High',
}

interface SelectChangeEvent {
  name?: string | undefined;
  value: unknown;
}

interface SeverityIndicatoryProps {
  severity: Severity;
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
    gridTemplateColumns: 'repeat(2, 1fr) 4fr 1fr',
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
  removeNotificationIcon: {
    color: theme.palette.error.main,
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

  const getStatusLabelClass = (severity: Severity): string => {
    switch (severity) {
      case Severity.High:
        return classes.high;
      case Severity.Medium:
        return classes.medium;
      case Severity.Low:
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

  const { showNotificationsDialog, onClose, notifications } = props;

  const [level, setLevel] = React.useState('');

  const [rmfNotifications, setRmfNotifications] = React.useState(notifications);

  // list of alert level for filtering
  // TODO - Check and change alert level once backend is up and confirm
  const alertLevel = ['Low', 'Medium', 'High', 'All'];

  React.useEffect(() => {
    setRmfNotifications(notifications);
  }, [notifications]);

  // handle filter change
  const handleChange = (e: React.ChangeEvent<SelectChangeEvent>) => {
    const val = e.target.value as string;
    setLevel(val);
    const filterNotifications: Notification[] = [];

    if (val === 'All') {
      setRmfNotifications(notifications);
    } else {
      notifications.forEach((notification) => {
        if (notification.severity === val) {
          filterNotifications.push(notification);
        }
      });
      setRmfNotifications(filterNotifications);
    }
  };

  return (
    <Dialog
      open={showNotificationsDialog}
      onClose={() => onClose()}
      fullWidth={true}
      maxWidth={'md'}
    >
      <DialogTitle>
        Notifications
        <IconButton aria-label="close" className={classes.closeButton} onClick={() => onClose()}>
          <CloseIcon />
        </IconButton>
      </DialogTitle>
      <DialogContent className={classes.dialogContent} dividers>
        <React.Fragment>
          <div className={classes.filter}>
            <Select
              className={classes.select}
              displayEmpty={true}
              value={level}
              onChange={(e) => handleChange(e)}
              input={<Input aria-label="filter-input" />}
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
          </div>

          {rmfNotifications.map((notification, i) => {
            return (
              <React.Fragment key={notification.time + '_' + i}>
                <Paper elevation={3} className={classes.paper}>
                  <SeverityIndicator
                    className={classes.indicator}
                    severity={notification.severity}
                  />
                  <Typography variant="body1" align="left">
                    {moment(notification.time, 'MMMM Do YYYY, h:mm:ss').fromNow()}
                  </Typography>
                  <Typography variant="body1" align="left">
                    {notification.error}
                  </Typography>
                  <Typography align="right">
                    {/* TODO - add function call to remove notification once backend is up */}
                    <IconButton className={classes.removeNotificationIcon}>
                      <CloseIcon />
                    </IconButton>
                  </Typography>
                </Paper>
              </React.Fragment>
            );
          })}
        </React.Fragment>
      </DialogContent>
      <DialogActions className={classes.dialogActions}>
        <Button autoFocus onClick={() => onClose()} color="primary">
          CLOSE
        </Button>
      </DialogActions>
    </Dialog>
  );
};
