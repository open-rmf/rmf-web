import React from 'react';
import {
  Button,
  Radio,
  RadioGroup,
  FormControlLabel,
  FormControl,
  FormLabel,
  Typography,
  Paper,
  makeStyles,
  Snackbar,
} from '@material-ui/core';
import MuiAlert from '@material-ui/lab/Alert';
import { TaskType as RmfTaskType } from 'rmf-models';
import type { SubmitTask, LoopTaskDescription } from 'api-client';
import { currentLocation, taskApi } from '../app-config';

const useStyles = makeStyles((theme) => ({
  root: {
    padding: theme.spacing(4),
    height: '100vh',
  },
  formLabel: {
    marginRight: 'auto',
  },
}));

class AlertType {
  static readonly SUCCESS = 'success';
  static readonly ERROR = 'error';
}

interface TaskFormProps {
  placeNames: string[];
}

export const TaskForm = (props: TaskFormProps) => {
  const classes = useStyles();
  const { placeNames } = props;
  const [place, setPlace] = React.useState('');
  const [showSnackBar, setShowSnackBar] = React.useState(false);
  const [alertType, setAlertType] = React.useState(AlertType.SUCCESS);

  const handleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setPlace(event.target.value);
  };

  const submitTask = () => {
    if (currentLocation && place) {
      const loopTaskDesc: LoopTaskDescription = {
        num_loops: 1,
        start_name: currentLocation,
        finish_name: place,
      };
      const task: SubmitTask = {
        task_type: RmfTaskType.TYPE_LOOP,
        start_time: new Date().getTime() / 1000,
        description: loopTaskDesc,
      };
      taskApi.submitTaskTasksSubmitTaskPost(task).then((res) => {
        if (res.data.task_id) {
          setAlertType(AlertType.SUCCESS);
          setShowSnackBar(true);
        }
      });
    } else {
      setAlertType(AlertType.ERROR);
      setShowSnackBar(true);
    }
  };

  return (
    <Paper className={classes.root}>
      <Typography variant="h5">Loading Bay</Typography>
      <FormControl component="fieldset">
        <FormLabel component="legend">Destination</FormLabel>
        <RadioGroup onChange={handleChange} row>
          {placeNames.map((p) => (
            <FormControlLabel
              key={p}
              value={p}
              disabled={p === currentLocation}
              control={<Radio />}
              label={p}
            />
          ))}
        </RadioGroup>
      </FormControl>
      <Typography variant="h6">Current Location - {currentLocation}</Typography>
      <Button variant="contained" color="primary" size="large" onClick={() => submitTask()}>
        Submit
      </Button>
      <Snackbar autoHideDuration={5000} open={showSnackBar} onClose={() => setShowSnackBar(false)}>
        {alertType === AlertType.ERROR ? (
          <MuiAlert severity="error">You must select a destination</MuiAlert>
        ) : (
          <MuiAlert severity="success">Task Submitted Successfully</MuiAlert>
        )}
      </Snackbar>
    </Paper>
  );
};
