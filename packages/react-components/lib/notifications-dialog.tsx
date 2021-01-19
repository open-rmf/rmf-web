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
import FiberManualRecordIcon from '@material-ui/icons/FiberManualRecord';

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
    display: 'flex',
    flexBasis: 0,
    flexGrow: 1,
    padding: theme.spacing(1),
    width: '100%',
    margin: `0.5rem 0`,
  },
  filter: {
    display: 'flex',
    float: 'right',
  },
  select: {
    width: '150px',
    padding: '0 0.5rem',
  },
  checkIcon: {
    color: theme.palette.success.main,
  },
}));

const severityStyles = makeStyles((theme) => ({
  high: {
    color: theme.palette.secondary.main,
  },
  medium: {
    color: theme.palette.error.main,
  },
  low: {
    color: theme.palette.warning.main,
  },
}));

interface SeverityIndicatoryProps {
  severity: string;
}

const SeverityIndicator = (props: SeverityIndicatoryProps): JSX.Element => {
  const { severity } = props;
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

  return <FiberManualRecordIcon className={getStatusLabelClass(severity)} />;
};

export const NotificationsDialog = (props: NotificationDialogProps): JSX.Element => {
  const classes = useStyles();

  const { showNotificationsDialog, setShowNotifications, notifications } = props;

  const [level, setLevel] = React.useState('');
  const [rmfNotifications, setRmfNotifications] = React.useState(notifications);

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
    const filterNotifications: { [key: string]: string }[] = [];

    notifications.forEach((notification) => {
      if (notification.severity === val) {
        filterNotifications.push(notification);
      }
    });
    setRmfNotifications(filterNotifications);
  };

  const restoreNotifications = () => {
    setRmfNotifications(notifications);
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
            renderValue={() => (level === '' ? <em>placeholder</em> : level)}
          >
            <MenuItem disabled value="">
              <em>Placeholder</em>
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
                  <Typography variant="body1">{notification.time}:</Typography>
                  <Typography variant="body1">{notification.error}</Typography>
                  <CheckIcon className={classes.checkIcon} />
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
