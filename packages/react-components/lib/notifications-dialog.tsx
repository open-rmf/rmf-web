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
  Grid,
} from '@material-ui/core';
import CloseIcon from '@material-ui/icons/Close';
import RestoreIcon from '@material-ui/icons/Restore';
import CheckIcon from '@material-ui/icons/Check';
import FiberManualRecordIcon from '@material-ui/icons/FiberManualRecord';

export interface Notification {
  time: string;
  error: string;
  severity: string;
}

export interface NotificationDialogProps {
  showNotificationsDialog: boolean;
  setShowNotifications: (payload: boolean) => void;
  notifications: Notification[];
  updateNotifications: (payload: Notification[]) => void;
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
    gridTemplateColumns: '2rem repeat(2, 1fr) 2rem',
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
}));

const severityStyles = makeStyles((theme) => ({
  high: {
    color: theme.palette.secondary.dark,
    padding: '0',
  },
  medium: {
    color: theme.palette.error.main,
    padding: '0',
  },
  low: {
    color: theme.palette.warning.light,
    padding: '0',
  },
}));

interface SeverityIndicatoryProps {
  severity: string;
  className?: string;
}

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
  return <FiberManualRecordIcon className={styles} />;
};

export const NotificationsDialog = (props: NotificationDialogProps): JSX.Element => {
  const classes = useStyles();

  const {
    showNotificationsDialog,
    setShowNotifications,
    notifications,
    updateNotifications,
  } = props;

  const [level, setLevel] = React.useState('');
  const [rmfNotifications, setRmfNotifications] = React.useState(notifications);
  const [notficationsCopy, setNotificationsCopy] = React.useState(notifications);

  const alertLevel = React.useMemo(() => {
    const holder: string[] = [];
    notifications.forEach((notification) => {
      if (!holder.includes(notification.severity)) {
        holder.push(notification.severity);
      }
    });
    return holder;
  }, [notifications]);

  const handleChange = (e: any) => {
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

  const restoreNotifications = () => {
    setRmfNotifications(notficationsCopy);
  };

  const deleteReadNotifications = (i: number) => {
    const beforeIndex = rmfNotifications.slice(0, i);
    const afterIndex = rmfNotifications.slice(i + 1);
    const newNotifications = beforeIndex.concat(afterIndex);
    setRmfNotifications(newNotifications);
    setNotificationsCopy(newNotifications);
    updateNotifications(newNotifications);
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
        <div className={classes.filter}>
          <Select
            className={classes.select}
            displayEmpty={true}
            value={level}
            onChange={(e) => handleChange(e)}
            input={<Input />}
            renderValue={() => (level === '' ? <em>Filter by severity</em> : level)}
          >
            <MenuItem disabled value="">
              <em>Filter by severity</em>
            </MenuItem>
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

        {rmfNotifications.length > 0 ? (
          rmfNotifications.map((notification, i) => {
            return (
              <React.Fragment key={notification.time + '_' + i}>
                <Paper elevation={3} className={classes.paper}>
                  <SeverityIndicator severity={notification.severity} />
                  <Typography variant="body1">{notification.time}</Typography>
                  <Typography variant="body1">{notification.error}</Typography>
                  <IconButton
                    className={classes.checkIcon}
                    onClick={() => deleteReadNotifications(i)}
                  >
                    <CheckIcon />
                  </IconButton>
                </Paper>
              </React.Fragment>
            );
          })
        ) : (
          <Typography variant="body1">No Notifications at the moment</Typography>
        )}
        <Typography variant="h6">Legend</Typography>
        <div className={classes.legend}>
          <Grid container direction="row" alignItems="center">
            <SeverityIndicator severity={'High'} /> <Typography variant="body1">High</Typography>
          </Grid>
          <Grid container direction="row" alignItems="center">
            <SeverityIndicator severity={'Medium'} />{' '}
            <Typography variant="body1">Medium</Typography>
          </Grid>
          <Grid container direction="row" alignItems="center">
            <SeverityIndicator severity={'Low'} /> <Typography variant="body1">Low</Typography>
          </Grid>
        </div>
      </DialogContent>
      <DialogActions className={classes.dialogActions}>
        <Button autoFocus onClick={() => setShowNotifications(false)} color="primary">
          CLOSE
        </Button>
      </DialogActions>
    </Dialog>
  );
};
