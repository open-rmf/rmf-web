import {
  Button,
  CircularProgress,
  Divider,
  Grid,
  List,
  ListItem,
  ListItemText,
  makeStyles,
  MenuItem,
  TextField,
  Typography,
  useTheme,
} from '@material-ui/core';
import Autocomplete from '@material-ui/lab/Autocomplete';
import React from 'react';
import * as RmfModels from 'rmf-models';
import type {
  CleanTaskDescription,
  DeliveryTaskDescription,
  LoopTaskDescription,
  SubmitTask,
} from 'api-client';
import DateFnsUtils from '@date-io/date-fns';
import { MuiPickersUtilsProvider, KeyboardDateTimePicker } from '@material-ui/pickers';
import { PositiveIntField } from '..';

const useStyles = makeStyles((theme) => ({
  taskList: {
    flex: '1 1 auto',
    minHeight: 400,
    maxHeight: '50vh',
    overflow: 'auto',
  },
  selectedTask: {
    background: theme.palette.action.focus,
  },
}));

function getShortDescription(task: SubmitTask): string {
  switch (task.task_type) {
    case RmfModels.TaskType.TYPE_CLEAN: {
      const desc: CleanTaskDescription = task.description;
      return `[Clean] zone [${desc.cleaning_zone}]`;
    }
    case RmfModels.TaskType.TYPE_DELIVERY: {
      const desc: DeliveryTaskDescription = task.description;
      return `[Delivery] from [${desc.pickup_place_name}] to [${desc.dropoff_place_name}]`;
    }
    case RmfModels.TaskType.TYPE_LOOP: {
      const desc: LoopTaskDescription = task.description;
      return `[Loop] from [${desc.start_name}] to [${desc.finish_name}]`;
    }
    default:
      return `[Unknown] type ${task.task_type}`;
  }
}

type TaskDescription = CleanTaskDescription | LoopTaskDescription | DeliveryTaskDescription;

interface DeliveryTaskFormProps {
  taskDesc: DeliveryTaskDescription;
  deliveryWaypoints: string[];
  dispensers: string[];
  ingestors: string[];
  onChange(deliveryTaskDescription: DeliveryTaskDescription): void;
}

function DeliveryTaskForm({
  taskDesc,
  deliveryWaypoints,
  dispensers,
  ingestors,
  onChange,
}: DeliveryTaskFormProps) {
  const theme = useTheme();

  return (
    <>
      <Grid container wrap="nowrap">
        <Grid item xs={12}>
          <Autocomplete
            id="pickup-location"
            freeSolo
            fullWidth
            options={deliveryWaypoints}
            value={taskDesc.pickup_place_name}
            onChange={(_ev, newValue) =>
              onChange({
                ...taskDesc,
                pickup_place_name: newValue,
              })
            }
            onBlur={(ev) =>
              onChange({ ...taskDesc, pickup_place_name: (ev.target as HTMLInputElement).value })
            }
            renderInput={(params) => (
              <TextField {...params} label="Pickup Location" margin="normal" />
            )}
          />
          <Autocomplete
            id="dispenser"
            freeSolo
            fullWidth
            options={dispensers}
            value={taskDesc.pickup_dispenser}
            onChange={(_ev, newValue) =>
              onChange({
                ...taskDesc,
                pickup_dispenser: newValue,
              })
            }
            onBlur={(ev) =>
              onChange({ ...taskDesc, pickup_dispenser: (ev.target as HTMLInputElement).value })
            }
            renderInput={(params) => <TextField {...params} label="Dispenser" margin="normal" />}
          />
          <Autocomplete
            id="dropoff-location"
            freeSolo
            fullWidth
            options={deliveryWaypoints}
            value={taskDesc.dropoff_place_name}
            onChange={(_ev, newValue) =>
              onChange({
                ...taskDesc,
                dropoff_place_name: newValue,
              })
            }
            onBlur={(ev) =>
              onChange({ ...taskDesc, dropoff_place_name: (ev.target as HTMLInputElement).value })
            }
            renderInput={(params) => (
              <TextField {...params} label="Dropoff Location" margin="normal" />
            )}
          />
          <Autocomplete
            id="ingestor"
            freeSolo
            fullWidth
            options={ingestors}
            value={taskDesc.dropoff_ingestor}
            onChange={(_ev, newValue) =>
              onChange({
                ...taskDesc,
                dropoff_ingestor: newValue,
              })
            }
            onBlur={(ev) =>
              onChange({ ...taskDesc, dropoff_ingestor: (ev.target as HTMLInputElement).value })
            }
            renderInput={(params) => <TextField {...params} label="Ingestor" margin="normal" />}
          />
        </Grid>
      </Grid>
    </>
  );
}

function defaultDeliveryTask(): DeliveryTaskDescription {
  return {
    pickup_place_name: '',
    pickup_dispenser: '',
    dropoff_place_name: '',
    dropoff_ingestor: '',
  };
}

function defaultTaskDescription(taskType?: number): TaskDescription | undefined {
  switch (taskType) {
    case RmfModels.TaskType.TYPE_DELIVERY:
      return defaultDeliveryTask();
    default:
      return undefined;
  }
}

function defaultTask(): SubmitTask {
  return {
    description: defaultDeliveryTask(),
    start_time: Math.floor(Date.now() / 1000),
    task_type: -1,
    priority: 0,
  };
}

export interface CustomFormProps {
  deliveryWaypoints?: string[];
  dispensers?: string[];
  ingestors?: string[];
  submitTasks?(tasks: SubmitTask[]): Promise<void>;
  onSuccess?(tasks: SubmitTask[]): void;
  onFail?(error: Error, tasks: SubmitTask[]): void;
}

export function CustomForm({
  deliveryWaypoints = [],
  dispensers = [],
  ingestors = [],
  submitTasks,
  onSuccess,
  onFail,
}: CustomFormProps): JSX.Element {
  const theme = useTheme();
  const classes = useStyles();
  const [tasks, setTasks] = React.useState<SubmitTask[]>(() => [defaultTask()]);
  const [selectedTaskIdx, setSelectedTaskIdx] = React.useState(0);
  const [priorityInput, setPriorityInput] = React.useState('0');
  const taskTitles = React.useMemo(
    () => tasks && tasks.map((t, i) => `${i + 1}: ${getShortDescription(t)}`),
    [tasks],
  );
  const [submitting, setSubmitting] = React.useState(false);
  const task = tasks[selectedTaskIdx];

  const updateTasks = () => {
    setTasks((prev) => {
      prev.splice(selectedTaskIdx, 1, task);
      return [...prev];
    });
  };

  const handleTaskDescriptionChange = (newType: number, newDesc: TaskDescription) => {
    task.task_type = newType;
    task.description = newDesc;
    updateTasks();
  };

  const renderTaskDescriptionForm = () => {
    if (task.task_type === -1) {
      return null;
    }
    switch (task.task_type) {
      case RmfModels.TaskType.TYPE_DELIVERY:
        return (
          <DeliveryTaskForm
            taskDesc={task.description as DeliveryTaskDescription}
            deliveryWaypoints={deliveryWaypoints}
            dispensers={dispensers}
            ingestors={ingestors}
            onChange={(desc) => handleTaskDescriptionChange(RmfModels.TaskType.TYPE_DELIVERY, desc)}
          />
        );
      default:
        return null;
    }
  };

  const handleTaskTypeChange = (ev: React.ChangeEvent<HTMLInputElement>) => {
    const newType = parseInt(ev.target.value);
    const newDesc = defaultTaskDescription(newType);
    if (newDesc === undefined) {
      return;
    }
    task.description = newDesc;
    task.task_type = newType;
    updateTasks();
  };

  const handleSubmit: React.MouseEventHandler<HTMLButtonElement> = (ev) => {
    ev.preventDefault();
    (async () => {
      if (!submitTasks) {
        onSuccess && onSuccess(tasks);
        return;
      }
      setSubmitting(true);
      try {
        setSubmitting(true);
        await submitTasks(tasks);
        onSuccess && onSuccess(tasks);
      } catch (e) {
        onFail && onFail(e, tasks);
      } finally {
        setSubmitting(false);
      }
    })();
  };

  return (
    <MuiPickersUtilsProvider utils={DateFnsUtils}>
      <Grid container direction="row" wrap="nowrap">
        <Grid>
          <TextField
            select
            id="task-type"
            label="Task Type"
            variant="outlined"
            fullWidth
            margin="normal"
            value={task.task_type !== -1 ? task.task_type : ''}
            onChange={handleTaskTypeChange}
          >
            <MenuItem value={RmfModels.TaskType.TYPE_DELIVERY}>Delivery</MenuItem>
          </TextField>
          <PositiveIntField
            id="priority"
            label="Priority"
            margin="normal"
            value={priorityInput}
            onChange={(ev) => {
              task.priority = parseInt(ev.target.value) || 0;
              updateTasks();
              setPriorityInput(ev.target.value);
            }}
          />
          <Grid container wrap="nowrap">
            <Grid style={{ flexGrow: 1 }}>
              <KeyboardDateTimePicker
                id="start-time"
                value={new Date(task.start_time * 1000)}
                onChange={(date) => {
                  if (!date) {
                    return;
                  }
                  // FIXME: needed because dateio typings default to moment
                  task.start_time = Math.floor(((date as unknown) as Date).getTime() / 1000);
                  updateTasks();
                }}
                label="Start Time"
                margin="dense"
                fullWidth
              />
            </Grid>
          </Grid>
          {renderTaskDescriptionForm()}
        </Grid>
      </Grid>
      <Grid>
        <Button
          style={{ margin: theme.spacing(1), maxWidth: '50%' }}
          type="submit"
          variant="contained"
          color="primary"
          disabled={submitting}
          onClick={handleSubmit}
          aria-label={'submit-btn'}
        >
          <Typography style={{ visibility: submitting ? 'hidden' : 'visible' }} variant="button">
            Submit
          </Typography>
          <CircularProgress
            style={{ position: 'absolute', visibility: submitting ? 'visible' : 'hidden' }}
            color="inherit"
            size="1.8em"
          />
        </Button>
      </Grid>
    </MuiPickersUtilsProvider>
  );
}
