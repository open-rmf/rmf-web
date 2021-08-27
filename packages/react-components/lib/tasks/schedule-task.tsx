import DateFnsUtils from '@date-io/date-fns';
import { Grid, makeStyles, MenuItem, TextField, useTheme } from '@material-ui/core';
import { KeyboardDateTimePicker, MuiPickersUtilsProvider } from '@material-ui/pickers';
import type {
  CleanTaskDescription,
  DeliveryTaskDescription,
  LoopTaskDescription,
  SubmitTask,
} from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { TaskDescriptionForm } from '.';
import { ConfirmationDialog, ConfirmationDialogProps } from '../confirmation-dialog';
import { PositiveIntField } from '../form-inputs';
import { defaultTask, defaultTaskDescription } from './create-task';
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

export interface SubmitTaskSchedule {
  ruleName: string;
  dayOfWeek: number[];
  endDatetime: string | null;
  frequency: number;
  frequencyType: string;
  frequencyTypeCustom: string;
  task: SubmitTask;
}

export interface ScheduleTaskFormProps
  extends Omit<ConfirmationDialogProps, 'onConfirmClick' | 'toolbar'> {
  cleaningZones?: string[];
  loopWaypoints?: string[];
  deliveryWaypoints?: string[];
  dispensers?: string[];
  ingestors?: string[];
  submitTask?(tasks: SubmitTaskSchedule): Promise<void>;
  onSuccess?(tasks: SubmitTaskSchedule): void;
  onFail?(error: Error, tasks: SubmitTaskSchedule): void;
}

export function ScheduleTaskForm({
  cleaningZones = [],
  loopWaypoints = [],
  deliveryWaypoints = [],
  dispensers = [],
  ingestors = [],
  submitTask,
  onSuccess,
  onFail,
  ...otherProps
}: ScheduleTaskFormProps): JSX.Element {
  const theme = useTheme();
  const classes = useStyles();
  // const [taskState, setTaskState] = React.useState<SubmitTask>(() => defaultTask());
  const { state, dispatch } = useTaskReducer({
    [TaskActionType.FrequencyType]: RecurrenceType.DoesNotRepeat,
    [TaskActionType.FrequencyTypeCustom]: RecurrenceType.Daily,
    [TaskActionType.Frequency]: 1,
    [TaskActionType.DayOfWeek]: [],
    [TaskActionType.EndDatetime]: null,
    [TaskActionType.RuleName]: '',
    [TaskActionType.Task]: defaultTask(),
  });

  const [submitting, setSubmitting] = React.useState(false);
  const task = defaultTask();

  const updateTasks = () => {
    // setTaskState(task);
    dispatch.setTask(task);
  };

  const handleTaskDescriptionChange = (newType: number, newDesc: TaskDescription) => {
    task.task_type = newType;
    task.description = newDesc;
    updateTasks();
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
    if (!submitTask) {
      onSuccess && onSuccess(state);
      return;
    }
    setSubmitting(true);
    try {
      setSubmitting(true);
      await submitTask(state);
      setSubmitting(false);
      onSuccess && onSuccess(state);
    } catch (e) {
      setSubmitting(false);
      onFail && onFail(e, state);
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
              value={state.task.task_type !== -1 ? state.task.task_type : ''}
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
              </Grid>
            </Grid>
            <TaskDescriptionForm
              task={state.task}
              handleTaskDescriptionChange={handleTaskDescriptionChange}
              cleaningZones={cleaningZones}
              loopWaypoints={loopWaypoints}
              deliveryWaypoints={deliveryWaypoints}
              dispensers={dispensers}
              ingestors={ingestors}
            />
          </Grid>
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
