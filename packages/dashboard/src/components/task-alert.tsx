import {
  Box,
  Button,
  Grid,
  LinearProgress,
  LinearProgressProps,
  Theme,
  Typography,
  Divider,
  TextField,
  useTheme,
} from '@mui/material';
import React from 'react';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogTitle from '@mui/material/DialogTitle';
import { makeStyles, createStyles } from '@mui/styles';
import { RobotWithTask } from './task-alert-store';
import { Status, TaskState } from 'api-client';
import { RmfAppContext } from './rmf-app';
import { Subscription } from 'rxjs';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    title: {
      '&.MuiDialogTitle-root': {
        padding: theme.spacing(1),
        textAlign: 'center',
        fontWeight: 'bold',
      },
    },
    subtitle: {
      '&.MuiTypography-root': {
        fontWeight: 'bold',
      },
    },
    textField: {
      background: theme.palette.background.default,
    },
  }),
);
export interface AlertProps {
  robots: RobotWithTask[];
}

function LinearProgressWithLabel(props: LinearProgressProps & { value: number }) {
  return (
    <Box sx={{ display: 'flex', alignItems: 'center' }}>
      <Box sx={{ width: '100%', mr: 1 }}>
        <LinearProgress variant="determinate" {...props} />
      </Box>
      <Box sx={{ minWidth: 35 }}>
        <Typography variant="body2" color="text.secondary">{`${Math.round(
          props.value * 100,
        )}%`}</Typography>
      </Box>
    </Box>
  );
}

interface AlertToDisplay extends RobotWithTask {
  show: boolean;
}

export function AlertComponent({ robots }: AlertProps): JSX.Element {
  const rmf = React.useContext(RmfAppContext);
  const classes = useStyles();
  const theme = useTheme();
  const [index, setIndex] = React.useState(0);
  const [alertToDisplay, setAlertToDisplay] = React.useState<AlertToDisplay[]>([]);

  const setDialogColor = (taskStatus: Status | undefined) => {
    if (taskStatus === undefined) return;

    switch (taskStatus) {
      case Status.Failed:
        return theme.palette.error.dark;

      case Status.Completed:
        // TODO
        // Add this color in the common-theme
        return '#6dff6f';

      default:
        return;
    }
  };

  const showMessage = (task: TaskState) => {
    switch (task.status) {
      case Status.Failed:
        return `${task.dispatch?.status} 
                ${task.dispatch?.errors?.map((e) => e.message)}`;

      case Status.Completed:
        return 'Task completed!';

      default:
        return 'No message';
    }
  };

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const subs: Subscription[] = [];

    robots.map((r) => {
      if (r.robot?.task_id) {
        return subs.push(
          rmf.getTaskStateObs(r.robot.task_id).subscribe((task) => {
            if (task.status === Status.Completed || task.status === Status.Failed) {
              setAlertToDisplay((prev) => [...prev, { show: true, task: task, robot: r.robot }]);
            }
          }),
        );
      }
      return [];
    });

    return () => subs.forEach((s) => s.unsubscribe());
  }, [rmf, robots]);

  const showTaskCompleteOrFailed = (current: AlertToDisplay) => {
    return (
      <Dialog
        PaperProps={{
          style: {
            backgroundColor: setDialogColor(current.task?.status),
            boxShadow: 'none',
          },
        }}
        open={current.show}
        maxWidth="xs"
      >
        <DialogTitle className={classes.title}>Alert</DialogTitle>
        <Divider />
        <DialogTitle className={classes.title}>Task State</DialogTitle>
        <Box sx={{ width: '100%' }}>
          <LinearProgressWithLabel value={current.robot?.battery ? current.robot?.battery : -1} />
        </Box>
        <DialogContent>
          <Grid container mb={1} alignItems="center" spacing={1}>
            <Grid item xs={3}>
              <Typography className={classes.subtitle}>Task</Typography>
            </Grid>
            <Grid item xs={9}>
              <TextField
                size="small"
                value={current.task?.booking.id}
                InputProps={{
                  readOnly: true,
                  className: classes.textField,
                }}
              />
            </Grid>
          </Grid>
          <Grid container mb={1} alignItems="center" spacing={1}>
            <Grid item xs={3}>
              <Typography className={classes.subtitle}>Location</Typography>
            </Grid>
            <Grid item xs={9}>
              <TextField
                size="small"
                value={current.robot?.location?.map}
                InputProps={{
                  readOnly: true,
                  className: classes.textField,
                }}
              />
            </Grid>
          </Grid>
          <Grid container alignItems="center" spacing={1}>
            <Grid item xs={3}>
              <Typography className={classes.subtitle}>Message</Typography>
            </Grid>
            <Grid item xs={9}>
              <TextField
                size="small"
                multiline
                value={current.task && showMessage(current.task)}
                maxRows={4}
                InputProps={{
                  readOnly: true,
                  className: classes.textField,
                }}
              />
            </Grid>
          </Grid>
        </DialogContent>
        <DialogActions>
          <Button
            onClick={() => {
              const clone = [...alertToDisplay];
              clone[index].show = false;
              setAlertToDisplay(clone);
              setIndex(index + 1);
            }}
            autoFocus
          >
            Close
          </Button>
        </DialogActions>
      </Dialog>
    );
  };

  return (
    <>
      {alertToDisplay[index] && alertToDisplay[index].show
        ? showTaskCompleteOrFailed(alertToDisplay[index])
        : null}
    </>
  );
}
