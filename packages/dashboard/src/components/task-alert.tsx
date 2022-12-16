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
} from '@mui/material';
import React from 'react';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogTitle from '@mui/material/DialogTitle';
import { makeStyles, createStyles } from '@mui/styles';
import { RobotAlert } from './task-alert-store';
import { Status } from 'api-client';
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
  }),
);
export interface AlertProps {
  robots: RobotAlert[];
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

export function AlertComponentStore({ robots }: AlertProps): JSX.Element {
  const rmf = React.useContext(RmfAppContext);
  const classes = useStyles();
  const [count, setCount] = React.useState(0);
  const [completed, setCompleted] = React.useState(true);

  const handleClose = () => {
    setCount(count + 1);
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
            if (task.status === Status.Underway) {
              setCompleted(true);
            }
          }),
        );
      }
      return [];
    });

    return () => subs.forEach((s) => s.unsubscribe());
  }, [rmf, robots]);

  const showTaskComplete = (robot: RobotAlert) => {
    if (completed) {
      console.log('Se completó la tarea');
      return (
        <Dialog open={count < robots.length} onClose={handleClose} maxWidth="xs">
          <DialogTitle className={classes.title}>Alert</DialogTitle>
          <Divider />
          <DialogTitle className={classes.title}>Task State</DialogTitle>
          <Box sx={{ width: '100%' }}>
            <LinearProgressWithLabel value={robot.robot?.battery ? robot.robot?.battery : -1} />
          </Box>
          <DialogContent>
            <Grid container mb={1} alignItems="center" spacing={1}>
              <Grid item xs={3}>
                <Typography className={classes.subtitle}>Task</Typography>
              </Grid>
              <Grid item xs={9}>
                <TextField
                  size="small"
                  value={robot.task?.booking.id}
                  InputProps={{
                    readOnly: true,
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
                  value={robot.robot?.location?.map}
                  InputProps={{
                    readOnly: true,
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
                  value="AMR ready for pickup at CSSD LOT3; cart not found, please push cart in CSSD LOT3"
                  maxRows={4}
                  InputProps={{
                    readOnly: true,
                  }}
                />
              </Grid>
            </Grid>
          </DialogContent>
          <DialogActions>
            <Button onClick={handleClose} autoFocus>
              Close
            </Button>
          </DialogActions>
        </Dialog>
      );
    }
  };

  return <>{robots && count < robots.length ? showTaskComplete(robots[count]) : null}</>;
}
