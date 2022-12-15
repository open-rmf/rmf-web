import { Button, Grid, Theme, Typography } from '@mui/material';
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
    paper: { minWidth: theme.spacing(2) },
  }),
);
export interface AlertProps {
  robots: RobotAlert[];
}

export function AlertComponentStore({ robots }: AlertProps): JSX.Element {
  const rmf = React.useContext(RmfAppContext);
  const classes = useStyles();
  const [count, setCount] = React.useState(0);
  const [completed, setCompleted] = React.useState(false);

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
            if (task.status === Status.Completed) {
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
          <DialogTitle>Alert</DialogTitle>
          <DialogTitle>Task State</DialogTitle>
          <DialogContent>
            <Grid container>
              <Grid item xs={3}>
                <Typography>Task</Typography>
              </Grid>
              <Grid item xs={3}>
                <Typography>{robot.task?.booking.id}</Typography>
              </Grid>
            </Grid>
            <Grid container>
              <Grid item xs={3}>
                <Typography>Location</Typography>
              </Grid>
              <Grid item xs={3}>
                <Typography>{robot.robot?.location?.map}</Typography>
              </Grid>
            </Grid>
            <Grid container>
              <Grid item xs={3}>
                <Typography>Message</Typography>
              </Grid>
              <Grid item xs={9}>
                <Typography>
                  AMR ready for pickup at CSSD LOT3; cart not found, please push cart in CSSD LOT3
                </Typography>
              </Grid>
            </Grid>
          </DialogContent>
          <DialogActions>
            <Button onClick={handleClose}>Disagree</Button>
            <Button onClick={handleClose} autoFocus>
              Agree
            </Button>
          </DialogActions>
        </Dialog>
      );
    }
  };

  return <>{robots && count < robots.length ? showTaskComplete(robots[count]) : null}</>;
}
