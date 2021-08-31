import DateFnsUtils from '@date-io/date-fns';
import { Grid, makeStyles, MenuItem, TextField, useTheme } from '@material-ui/core';
import { KeyboardDateTimePicker, MuiPickersUtilsProvider } from '@material-ui/pickers';
import type {
  CleanTaskDescription,
  DeliveryTaskDescription,
  LoopTaskDescription,
} from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { TaskDescriptionForm } from '.';
import { ConfirmationDialog, ConfirmationDialogProps } from '../confirmation-dialog';
import { PositiveIntField } from '../form-inputs';
import { defaultTask, defaultTaskDescription } from './create-task';
import RecurrentRules, { RecurrenceType } from './scheduler/rules';
import { CustomTaskSchedule } from './scheduler/scheduler';
import { TaskActionType, TaskState, useTaskReducer } from './task-reducer';

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

export type SubmitTaskSchedule = TaskState;

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
    [TaskActionType.FrequencyType]: RecurrenceType.Once,
    [TaskActionType.FrequencyTypeCustom]: RecurrenceType.Daily,
    [TaskActionType.Frequency]: 1,
    [TaskActionType.DayOfWeek]: [],
    [TaskActionType.EndDatetime]: new Date(),
    [TaskActionType.RuleName]: '',
    [TaskActionType.StartDatetime]: new Date(),
    [TaskActionType.Task]: defaultTask(),
  });

  const frequencyOptionList = React.useMemo(
    () => RecurrentRules.getRecurrenceTypeList(state.startDatetime),
    [state.startDatetime],
  );

  const handleFrequencyTypeChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    dispatch.setFrequencyType(event.target.value as string);
  };

  const [submitting, setSubmitting] = React.useState(false);
  const task = defaultTask();

  const updateTasks = () => {
    dispatch.setTask(task);
  };

  const handleTaskDescriptionChange = (newType: number, newDesc: TaskDescription) => {
    task.task_type = newType;
    task.description = newDesc;
    updateTasks();
  };

  const handleRuleNameChange = (ev: React.ChangeEvent<HTMLInputElement>) => {
    dispatch.setRuleName(ev.target.value);
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

  const isFormValid = () => {
    // TODO add states and error messages in every input field
    let isValid = true;

    if (state.endDatetime) {
      if (state.endDatetime < state.startDatetime) {
        console.error('EndDatetime cannot be less than StartDatetime');

        isValid = false;
      }
    }

    if (state.endDatetime < new Date()) {
      isValid = false;
      console.error('endDatetime cannot be less than current date');
    }

    if (state.startDatetime < new Date()) {
      isValid = false;
      console.error('StartDatetime cannot be less than current date');
    }

    if (state.frequencyType === RecurrenceType.Custom) {
      if (state.frequency === 0) {
        isValid = false;
        console.error('Frequency cannot be empty');
      }
    }

    if (state.ruleName === '') {
      isValid = false;
      console.error('Rule Name cannot be empty');
    }

    return isValid;
  };
  // no memo because deps would likely change
  const handleSubmit: React.MouseEventHandler = async (ev) => {
    if (!isFormValid()) {
      console.error('Form not valid');
      return;
    }

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
                  value={state.startDatetime}
                  // value={new Date(task.start_time * 1000)}
                  onChange={(date) => {
                    if (!date) {
                      return;
                    }
                    // FIXME: needed because dateio typings default to moment
                    dispatch.setStartDatetime(date);
                    // task.start_time =   Math.floor(((date as unknown) as Date).getTime() / 1000),

                    // updateTasks();
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
          <br />
          <Grid style={{ flexGrow: 1 }}>
            <KeyboardDateTimePicker
              id="end-time"
              value={state.endDatetime || new Date()}
              onChange={(date) => {
                if (!date) {
                  return;
                }
                dispatch.setEndDatetime(date);
              }}
              label="Ends"
              margin="normal"
              fullWidth
            />
          </Grid>
        </Grid>
        <Grid container wrap="nowrap">
          <Grid style={{ flexGrow: 1 }}>
            <TextField
              id="rule-name"
              label="Rule Name"
              variant="outlined"
              fullWidth
              margin="normal"
              value={state.ruleName}
              onChange={handleRuleNameChange}
            ></TextField>
          </Grid>
          <Grid style={{ flexGrow: 1 }}>
            <TextField
              select
              id="frequency-type-picker"
              label="Task frequency"
              variant="outlined"
              fullWidth
              margin="normal"
              value={state.frequencyType}
              onChange={handleFrequencyTypeChange}
            >
              {frequencyOptionList.map((option) => (
                <MenuItem key={option.key} value={option.key as string}>
                  {option.value}
                </MenuItem>
              ))}
            </TextField>
            <br />
          </Grid>
        </Grid>
        <Grid container wrap="nowrap">
          <Grid style={{ flexGrow: 1 }}>
            {state.frequencyType === RecurrenceType.Custom && (
              <CustomTaskSchedule state={state} dispatch={dispatch} />
            )}
          </Grid>
        </Grid>
      </ConfirmationDialog>
    </MuiPickersUtilsProvider>
  );
}
