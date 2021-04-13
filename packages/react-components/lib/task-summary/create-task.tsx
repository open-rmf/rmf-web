import DateFnsUtils from '@date-io/date-fns';
import {
  Button,
  Divider,
  Grid,
  makeStyles,
  MenuItem,
  TextField,
  Typography,
  useTheme,
} from '@material-ui/core';
import { KeyboardDateTimePicker, MuiPickersUtilsProvider } from '@material-ui/pickers';
import type {
  CleanTaskDescription,
  LoopTaskDescription,
  DeliveryTaskDescription,
} from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';

type TaskDescription = CleanTaskDescription | LoopTaskDescription | DeliveryTaskDescription;

const useStyles = makeStyles((theme) => ({
  uploadFileBtn: {
    marginBottom: theme.spacing(1),
  },
  submitBtn: {
    alignSelf: 'flex-end',
    marginTop: theme.spacing(1),
  },
}));

function FormToolbar() {
  const classes = useStyles();

  return (
    <Grid container wrap="nowrap" justify="center">
      <Grid style={{ flexGrow: 1 }}>
        <Typography variant="h6">Create Task</Typography>
      </Grid>
      <Grid>
        <Button className={classes.uploadFileBtn} variant="contained" color="primary">
          Upload File
        </Button>
      </Grid>
    </Grid>
  );
}

interface DeliveryTaskFormProps {
  value: DeliveryTaskDescription;
  onChange(deliveryTaskDescription: DeliveryTaskDescription): void;
}

function DeliveryTaskForm({ value, onChange }: DeliveryTaskFormProps) {
  const theme = useTheme();

  return (
    <>
      <Grid container wrap="nowrap">
        <Grid style={{ flex: '1 1 60%' }}>
          <TextField
            label="Pickup Location"
            fullWidth
            margin="normal"
            value={value.pickupPlaceName}
            onChange={(ev) =>
              onChange({
                ...value,
                pickupPlaceName: ev.target.value,
              })
            }
          />
        </Grid>
        <Grid
          style={{
            flex: '1 1 40%',
            marginLeft: theme.spacing(2),
            marginRight: theme.spacing(2),
          }}
        >
          <TextField
            label="Dispenser"
            fullWidth
            margin="normal"
            value={value.pickupDispenser}
            onChange={(ev) => {
              onChange({
                ...value,
                pickupDispenser: ev.target.value,
              });
            }}
          />
        </Grid>
      </Grid>
      <Grid container wrap="nowrap">
        <Grid style={{ flex: '1 1 60%' }}>
          <TextField
            label="Dropoff Location"
            fullWidth
            margin="normal"
            value={value.dropoffPlaceName}
            onChange={(ev) =>
              onChange({
                ...value,
                dropoffPlaceName: ev.target.value,
              })
            }
          />
        </Grid>
        <Grid
          style={{
            flex: '1 1 40%',
            marginLeft: theme.spacing(2),
            marginRight: theme.spacing(2),
          }}
        >
          <TextField
            label="Ingestor"
            fullWidth
            margin="normal"
            value={value.dropoffIngestor}
            onChange={(ev) => {
              onChange({
                ...value,
                dropoffIngestor: ev.target.value,
              });
            }}
          />
        </Grid>
      </Grid>
    </>
  );
}

interface LoopTaskFormProps {
  value: LoopTaskDescription;
  onChange(loopTaskDescription: LoopTaskDescription): void;
}

function LoopTaskForm({ value, onChange }: LoopTaskFormProps) {
  const theme = useTheme();

  return (
    <>
      <TextField
        label="Start Location"
        fullWidth
        margin="normal"
        value={value.startName}
        onChange={(ev) =>
          onChange({
            ...value,
            startName: ev.target.value,
          })
        }
      />
      <Grid container wrap="nowrap">
        <Grid style={{ flex: '1 1 100%' }}>
          <TextField
            label="Finish Location"
            fullWidth
            margin="normal"
            value={value.startName}
            onChange={(ev) =>
              onChange({
                ...value,
                finishName: ev.target.value,
              })
            }
          />
        </Grid>
        <Grid
          style={{
            flex: '0 1 5em',
            marginLeft: theme.spacing(2),
            marginRight: theme.spacing(2),
          }}
        >
          <TextField
            error={isNaN(value.numLoops)}
            type="number"
            label="Loops"
            helperText={isNaN(value.numLoops) && 'Required'}
            margin="normal"
            value={value.numLoops}
            onChange={(ev) => {
              onChange({
                ...value,
                numLoops: parseInt(ev.target.value),
              });
            }}
          />
        </Grid>
      </Grid>
    </>
  );
}

interface CleanTaskFormProps {
  value: CleanTaskDescription;
  onChange(cleanTaskDescription: CleanTaskDescription): void;
}

function CleanTaskForm({ value, onChange }: CleanTaskFormProps) {
  return (
    <TextField
      label="Cleaning Zone"
      fullWidth
      margin="normal"
      value={value.cleaningZone}
      onChange={(ev) =>
        onChange({
          cleaningZone: ev.target.value,
        })
      }
    />
  );
}

interface TaskDescriptionFormProps {
  taskType: number;
  value: TaskDescription;
  onChange(taskDescription: TaskDescription): void;
}

function TaskDescriptionForm({ taskType, value, onChange }: TaskDescriptionFormProps) {
  switch (taskType) {
    case RmfModels.TaskType.TYPE_CLEAN:
      return <CleanTaskForm value={value as CleanTaskDescription} onChange={onChange} />;
    case RmfModels.TaskType.TYPE_LOOP:
      return <LoopTaskForm value={value as LoopTaskDescription} onChange={onChange} />;
    case RmfModels.TaskType.TYPE_DELIVERY:
      return <DeliveryTaskForm value={value as DeliveryTaskDescription} onChange={onChange} />;
    default:
      return null;
  }
}

function defaultCleanTask(): CleanTaskDescription {
  return {
    cleaningZone: '',
  };
}

function defaultLoopsTask(): LoopTaskDescription {
  return {
    startName: '',
    finishName: '',
    numLoops: 1,
  };
}

function defaultDeliveryTask(): DeliveryTaskDescription {
  return {
    pickupPlaceName: '',
    pickupDispenser: '',
    dropoffPlaceName: '',
    dropoffIngestor: '',
  };
}

function defaultTaskDescription(taskType?: number): TaskDescription | undefined {
  switch (taskType) {
    case RmfModels.TaskType.TYPE_CLEAN:
      return defaultCleanTask();
    case RmfModels.TaskType.TYPE_LOOP:
      return defaultLoopsTask();
    case RmfModels.TaskType.TYPE_DELIVERY:
      return defaultDeliveryTask();
    default:
      return undefined;
  }
}

export interface CreateTaskFormProps {}

export function CreateTaskForm() {
  const classes = useStyles();
  const theme = useTheme();
  const [taskType, setTaskType] = React.useState<number | undefined>(undefined);
  const [startDate, setStartDate] = React.useState(new Date());
  const [priority, setPriority] = React.useState(0);
  const [taskDescription, setTaskDescrption] = React.useState<TaskDescription | undefined>(() =>
    defaultTaskDescription(taskType),
  );

  const handleTaskTypeChange = React.useCallback((ev: React.ChangeEvent<HTMLInputElement>) => {
    const newType = parseInt(ev.target.value);
    setTaskDescrption(defaultTaskDescription(newType));
    setTaskType(newType);
  }, []);

  return (
    <Grid container direction="column" wrap="nowrap">
      <FormToolbar />
      <Divider />
      <Grid>
        <MuiPickersUtilsProvider utils={DateFnsUtils}>
          <TextField
            select
            label="Task Type"
            variant="outlined"
            fullWidth
            margin="normal"
            value={taskType}
            onChange={handleTaskTypeChange}
          >
            <MenuItem value={RmfModels.TaskType.TYPE_CLEAN}>Clean</MenuItem>
            <MenuItem value={RmfModels.TaskType.TYPE_LOOP}>Loop</MenuItem>
            <MenuItem value={RmfModels.TaskType.TYPE_DELIVERY}>Delivery</MenuItem>
          </TextField>
          <Grid container wrap="nowrap">
            <Grid style={{ flexGrow: 1 }}>
              <KeyboardDateTimePicker
                value={startDate}
                onChange={(date) => date && setStartDate((date as unknown) as Date)}
                label="Start Time"
                margin="normal"
                fullWidth
              />
            </Grid>
            <Grid
              style={{
                flex: '0 1 5em',
                marginLeft: theme.spacing(2),
                marginRight: theme.spacing(2),
              }}
            >
              <TextField
                error={isNaN(priority)}
                type="number"
                label="Priority"
                helperText={isNaN(priority) && 'Required'}
                margin="normal"
                value={priority}
                onChange={(ev) => setPriority(parseInt(ev.target.value))}
              />
            </Grid>
          </Grid>
          {taskType && taskDescription && (
            <TaskDescriptionForm
              taskType={taskType}
              value={taskDescription}
              onChange={setTaskDescrption}
            />
          )}
        </MuiPickersUtilsProvider>
      </Grid>
      <Grid className={classes.submitBtn}>
        <Button variant="contained" color="primary">
          Submit
        </Button>
      </Grid>
    </Grid>
  );
}
