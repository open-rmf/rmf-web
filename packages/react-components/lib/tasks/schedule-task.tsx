import DateFnsUtils from '@date-io/date-fns';
import {
  Button,
  Divider,
  Grid,
  List,
  ListItem,
  ListItemText,
  makeStyles,
  MenuItem,
  TextField,
  useTheme,
} from '@material-ui/core';
import { Autocomplete } from '@material-ui/lab';
import { KeyboardDateTimePicker, MuiPickersUtilsProvider } from '@material-ui/pickers';
import type {
  CleanTaskDescription,
  DeliveryTaskDescription,
  LoopTaskDescription,
  SubmitTask,
} from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { ConfirmationDialog, ConfirmationDialogProps } from '../confirmation-dialog';
import { PositiveIntField } from '../form-inputs';
import { CleanTaskForm, DeliveryTaskForm, LoopTaskForm } from './create-task';
import { RecurrenceType } from './scheduler/rules';
import Scheduler from './scheduler/scheduler';
import { TaskActionType, useTaskReducer } from './task-reducer';

type TaskDescription = CleanTaskDescription | LoopTaskDescription | DeliveryTaskDescription;

const useStyles = makeStyles((theme) => ({
  selectFileBtn: {
    marginBottom: theme.spacing(1),
  },
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

function defaultCleanTask(): CleanTaskDescription {
  return {
    cleaning_zone: '',
  };
}

function defaultLoopsTask(): LoopTaskDescription {
  return {
    start_name: '',
    finish_name: '',
    num_loops: 1,
  };
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

function defaultTask(): SubmitTask {
  return {
    description: defaultCleanTask(),
    start_time: Math.floor(Date.now() / 1000),
    task_type: -1,
    priority: 0,
  };
}

export interface ScheduleTaskFormProps
  extends Omit<ConfirmationDialogProps, 'onConfirmClick' | 'toolbar'> {
  /**
   * Shows extra UI elements suitable for submittng batched tasks. Default to 'false'.
   */
  allowBatch?: boolean;
  cleaningZones?: string[];
  loopWaypoints?: string[];
  deliveryWaypoints?: string[];
  dispensers?: string[];
  ingestors?: string[];
  submitTasks?(tasks: SubmitTask[]): Promise<void>;
  tasksFromFile?(): Promise<SubmitTask[]> | SubmitTask[];
  onSuccess?(tasks: SubmitTask[]): void;
  onFail?(error: Error, tasks: SubmitTask[]): void;
}

export function ScheduleTaskForm({
  cleaningZones = [],
  loopWaypoints = [],
  deliveryWaypoints = [],
  dispensers = [],
  ingestors = [],
  submitTasks,
  onSuccess,
  onFail,
  ...otherProps
}: ScheduleTaskFormProps): JSX.Element {
  const theme = useTheme();
  const classes = useStyles();
  const [tasks, setTasks] = React.useState<SubmitTask[]>(() => [defaultTask()]);
  const [selectedTaskIdx, setSelectedTaskIdx] = React.useState(0);
  const { state, dispatch } = useTaskReducer({
    [TaskActionType.FrequencyType]: RecurrenceType.DoesNotRepeat,
    [TaskActionType.FrequencyTypeCustom]: RecurrenceType.Daily,
    [TaskActionType.Frequency]: 1,
    [TaskActionType.DayOfWeek]: [],
    [TaskActionType.EndDatetime]: null,
  });

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
      case RmfModels.TaskType.TYPE_CLEAN:
        return (
          <CleanTaskForm
            taskDesc={task.description as CleanTaskDescription}
            cleaningZones={cleaningZones}
            onChange={(desc) => handleTaskDescriptionChange(RmfModels.TaskType.TYPE_CLEAN, desc)}
          />
        );
      case RmfModels.TaskType.TYPE_LOOP:
        return (
          <LoopTaskForm
            taskDesc={task.description as LoopTaskDescription}
            loopWaypoints={loopWaypoints}
            onChange={(desc) => handleTaskDescriptionChange(RmfModels.TaskType.TYPE_LOOP, desc)}
          />
        );
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

  // no memo because deps would likely change
  const handleSubmit: React.MouseEventHandler = async (ev) => {
    ev.preventDefault();
    if (!submitTasks) {
      onSuccess && onSuccess(tasks);
      return;
    }
    setSubmitting(true);
    try {
      setSubmitting(true);
      await submitTasks(tasks);
      setSubmitting(false);
      onSuccess && onSuccess(tasks);
    } catch (e) {
      setSubmitting(false);
      onFail && onFail(e, tasks);
    }
  };

  return (
    <MuiPickersUtilsProvider utils={DateFnsUtils}>
      <ConfirmationDialog
        title="Schedule Task"
        submitting={submitting}
        confirmText={'Schedule'}
        maxWidth="md"
        fullWidth={false}
        // toolbar={}
        onSubmit={handleSubmit}
        {...otherProps}
      >
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
              <MenuItem value={RmfModels.TaskType.TYPE_CLEAN}>Clean</MenuItem>
              <MenuItem value={RmfModels.TaskType.TYPE_LOOP}>Loop</MenuItem>
              <MenuItem value={RmfModels.TaskType.TYPE_DELIVERY}>Delivery</MenuItem>
            </TextField>
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
                <PositiveIntField
                  id="priority"
                  label="Priority"
                  margin="normal"
                  value={task.priority || 0}
                  onChange={(_ev, val) => {
                    task.priority = val;
                    updateTasks();
                  }}
                />

                {renderTaskDescriptionForm()}
              </Grid>
            </Grid>
            {renderTaskDescriptionForm()}
          </Grid>
          {taskTitles.length > 1 && (
            <>
              <Divider
                orientation="vertical"
                flexItem
                style={{ marginLeft: theme.spacing(2), marginRight: theme.spacing(2) }}
              />
              <List dense className={classes.taskList} aria-label="Tasks List">
                {taskTitles.map((title, idx) => (
                  <ListItem
                    key={idx}
                    button
                    onClick={() => setSelectedTaskIdx(idx)}
                    className={selectedTaskIdx === idx ? classes.selectedTask : undefined}
                    role="listitem button"
                  >
                    <ListItemText primary={title} />
                  </ListItem>
                ))}
              </List>
            </>
          )}
        </Grid>
        <Grid container wrap="nowrap">
          <Grid style={{ flexGrow: 1 }}>
            <Scheduler
              selectedDate={new Date(task.start_time * 1000)}
              state={state}
              dispatch={dispatch}
            ></Scheduler>
          </Grid>
        </Grid>
      </ConfirmationDialog>
    </MuiPickersUtilsProvider>
  );
}
