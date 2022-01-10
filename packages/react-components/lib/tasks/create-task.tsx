import { DateTimePicker, LocalizationProvider } from '@mui/lab';
import AdapterDateFns from '@mui/lab/AdapterDateFns';
import {
  Autocomplete,
  Button,
  Divider,
  Grid,
  List,
  ListItem,
  ListItemText,
  MenuItem,
  styled,
  TextField,
  useTheme,
} from '@mui/material';
// import {
//   CleanTaskDescription,
//   DeliveryTaskDescription,
//   LoopTaskDescription,
//   SubmitTask,
// } from 'api-client';
import React from 'react';
import { TaskType as RmfTaskType } from 'rmf-models';
import { ConfirmationDialog, ConfirmationDialogProps } from '../confirmation-dialog';
import { PositiveIntField } from '../form-inputs';

type TaskDescription = any;

const classes = {
  selectFileBtn: 'create-task-selected-file-btn',
  taskList: 'create-task-task-list',
  selectedTask: 'create-task-selected-task',
};
const StyledConfirmationDialog = styled((props: ConfirmationDialogProps) => (
  <ConfirmationDialog {...props} />
))(({ theme }) => ({
  [`& .${classes.selectFileBtn}`]: {
    marginBottom: theme.spacing(1),
  },
  [`& .${classes.taskList}`]: {
    flex: '1 1 auto',
    minHeight: 400,
    maxHeight: '50vh',
    overflow: 'auto',
  },
  [`& .${classes.selectedTask}`]: {
    background: theme.palette.action.focus,
  },
}));

function getShortDescription(task: any): string {
  switch (task.task_type) {
    case RmfTaskType.TYPE_CLEAN: {
      const desc = task.description as any;
      return `[Clean] zone [${desc.cleaning_zone}]`;
    }
    case RmfTaskType.TYPE_DELIVERY: {
      const desc = task.description as any;
      return `[Delivery] from [${desc.pickup_place_name}] to [${desc.dropoff_place_name}]`;
    }
    case RmfTaskType.TYPE_LOOP: {
      const desc = task.description as any;
      return `[Loop] from [${desc.start_name}] to [${desc.finish_name}]`;
    }
    default:
      return `[Unknown] type ${task.task_type}`;
  }
}

interface FormToolbarProps {
  onSelectFileClick?: React.MouseEventHandler<HTMLButtonElement>;
}

function FormToolbar({ onSelectFileClick }: FormToolbarProps) {
  return (
    <Button
      aria-label="Select File"
      className={classes.selectFileBtn}
      variant="contained"
      color="primary"
      onClick={onSelectFileClick}
    >
      Select File
    </Button>
  );
}

interface DeliveryTaskFormProps {
  taskDesc: any;
  deliveryWaypoints: string[];
  dispensers: string[];
  ingestors: string[];
  onChange(deliveryTaskDescription: any): void;
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
        <Grid style={{ flex: '1 1 60%' }}>
          <Autocomplete
            id="pickup-location"
            freeSolo
            fullWidth
            options={deliveryWaypoints}
            value={taskDesc.pickup_place_name}
            onChange={(_ev, newValue) =>
              newValue !== null &&
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
        </Grid>
        <Grid
          style={{
            flex: '1 1 40%',
            marginLeft: theme.spacing(2),
            marginRight: theme.spacing(2),
          }}
        >
          <Autocomplete
            id="dispenser"
            freeSolo
            fullWidth
            options={dispensers}
            value={taskDesc.pickup_dispenser}
            onChange={(_ev, newValue) =>
              newValue !== null &&
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
        </Grid>
      </Grid>
      <Grid container wrap="nowrap">
        <Grid style={{ flex: '1 1 60%' }}>
          <Autocomplete
            id="dropoff-location"
            freeSolo
            fullWidth
            options={deliveryWaypoints}
            value={taskDesc.dropoff_place_name}
            onChange={(_ev, newValue) =>
              newValue !== null &&
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
        </Grid>
        <Grid
          style={{
            flex: '1 1 40%',
            marginLeft: theme.spacing(2),
            marginRight: theme.spacing(2),
          }}
        >
          <Autocomplete
            id="ingestor"
            freeSolo
            fullWidth
            options={ingestors}
            value={taskDesc.dropoff_ingestor}
            onChange={(_ev, newValue) =>
              newValue !== null &&
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

interface LoopTaskFormProps {
  taskDesc: any;
  loopWaypoints: string[];
  onChange(loopTaskDescription: any): void;
}

function LoopTaskForm({ taskDesc, loopWaypoints, onChange }: LoopTaskFormProps) {
  const theme = useTheme();

  return (
    <>
      <Autocomplete
        id="start-location"
        freeSolo
        fullWidth
        options={loopWaypoints}
        value={taskDesc.start_name}
        onChange={(_ev, newValue) =>
          newValue !== null &&
          onChange({
            ...taskDesc,
            start_name: newValue,
          })
        }
        onBlur={(ev) =>
          onChange({ ...taskDesc, start_name: (ev.target as HTMLInputElement).value })
        }
        renderInput={(params) => <TextField {...params} label="Start Location" margin="normal" />}
      />
      <Grid container wrap="nowrap">
        <Grid style={{ flex: '1 1 100%' }}>
          <Autocomplete
            id="finish-location"
            freeSolo
            fullWidth
            options={loopWaypoints}
            value={taskDesc.finish_name}
            onChange={(_ev, newValue) =>
              newValue !== null &&
              onChange({
                ...taskDesc,
                finish_name: newValue,
              })
            }
            onBlur={(ev) =>
              onChange({ ...taskDesc, finish_name: (ev.target as HTMLInputElement).value })
            }
            renderInput={(params) => (
              <TextField {...params} label="Finish Location" margin="normal" />
            )}
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
            id="loops"
            label="Loops"
            margin="normal"
            value={taskDesc.num_loops}
            onChange={(_ev, val) => {
              onChange({
                ...taskDesc,
                num_loops: val,
              });
            }}
          />
        </Grid>
      </Grid>
    </>
  );
}

interface CleanTaskFormProps {
  taskDesc: any;
  cleaningZones: string[];
  onChange(cleanTaskDescription: any): void;
}

function CleanTaskForm({ taskDesc, cleaningZones, onChange }: CleanTaskFormProps) {
  return (
    <Autocomplete
      id="cleaning-zone"
      freeSolo
      fullWidth
      options={cleaningZones}
      value={taskDesc.cleaning_zone}
      onChange={(_ev, newValue) =>
        newValue !== null &&
        onChange({
          ...taskDesc,
          cleaning_zone: newValue,
        })
      }
      onBlur={(ev) =>
        onChange({ ...taskDesc, cleaning_zone: (ev.target as HTMLInputElement).value })
      }
      renderInput={(params) => <TextField {...params} label="Cleaning Zone" margin="normal" />}
    />
  );
}

function defaultCleanTask(): any {
  return {
    cleaning_zone: '',
  };
}

function defaultLoopsTask(): any {
  return {
    start_name: '',
    finish_name: '',
    num_loops: 1,
  };
}

function defaultDeliveryTask(): any {
  return {
    pickup_place_name: '',
    pickup_dispenser: '',
    dropoff_place_name: '',
    dropoff_ingestor: '',
  };
}

function defaultTaskDescription(taskType?: number): TaskDescription | undefined {
  switch (taskType) {
    case RmfTaskType.TYPE_CLEAN:
      return defaultCleanTask();
    case RmfTaskType.TYPE_LOOP:
      return defaultLoopsTask();
    case RmfTaskType.TYPE_DELIVERY:
      return defaultDeliveryTask();
    default:
      return undefined;
  }
}

function defaultTask(): any {
  return {
    description: defaultCleanTask(),
    start_time: Math.floor(Date.now() / 1000),
    task_type: -1,
    priority: 0,
  };
}

export interface CreateTaskFormProps
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
  submitTasks?(tasks: any[]): Promise<void>;
  tasksFromFile?(): Promise<any[]> | any[];
  onSuccess?(tasks: any[]): void;
  onFail?(error: Error, tasks: any[]): void;
}

export function CreateTaskForm({
  cleaningZones = [],
  loopWaypoints = [],
  deliveryWaypoints = [],
  dispensers = [],
  ingestors = [],
  submitTasks,
  tasksFromFile,
  onSuccess,
  onFail,
  ...otherProps
}: CreateTaskFormProps): JSX.Element {
  const theme = useTheme();
  const [tasks, setTasks] = React.useState<any[]>(() => [defaultTask()]);
  const [selectedTaskIdx, setSelectedTaskIdx] = React.useState(0);
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
    if ((task.task_type as number) === -1) {
      return null;
    }
    switch (task.task_type) {
      case RmfTaskType.TYPE_CLEAN:
        return (
          <CleanTaskForm
            taskDesc={task.description as any}
            cleaningZones={cleaningZones}
            onChange={(desc) => handleTaskDescriptionChange(RmfTaskType.TYPE_CLEAN, desc)}
          />
        );
      case RmfTaskType.TYPE_LOOP:
        return (
          <LoopTaskForm
            taskDesc={task.description as any}
            loopWaypoints={loopWaypoints}
            onChange={(desc) => handleTaskDescriptionChange(RmfTaskType.TYPE_LOOP, desc)}
          />
        );
      case RmfTaskType.TYPE_DELIVERY:
        return (
          <DeliveryTaskForm
            taskDesc={task.description as any}
            deliveryWaypoints={deliveryWaypoints}
            dispensers={dispensers}
            ingestors={ingestors}
            onChange={(desc) => handleTaskDescriptionChange(RmfTaskType.TYPE_DELIVERY, desc)}
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
      onFail && onFail(e as Error, tasks);
    }
  };

  const handleSelectFileClick: React.MouseEventHandler<HTMLButtonElement> = () => {
    if (!tasksFromFile) {
      return;
    }
    (async () => {
      const newTasks = await tasksFromFile();
      if (newTasks.length === 0) {
        return;
      }
      setTasks(newTasks);
      setSelectedTaskIdx(0);
    })();
  };

  const submitText = tasks.length > 1 ? 'Submit All' : 'Submit';

  return (
    <LocalizationProvider dateAdapter={AdapterDateFns}>
      <StyledConfirmationDialog
        title="Create Task"
        submitting={submitting}
        confirmText={submitText}
        maxWidth="md"
        fullWidth={tasks.length > 1}
        toolbar={<FormToolbar onSelectFileClick={handleSelectFileClick} />}
        onSubmit={handleSubmit}
        disableEnforceFocus
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
              value={(task.task_type as number) !== -1 ? task.task_type : ''}
              onChange={handleTaskTypeChange}
            >
              <MenuItem value={RmfTaskType.TYPE_CLEAN}>Clean</MenuItem>
              <MenuItem value={RmfTaskType.TYPE_LOOP}>Loop</MenuItem>
              <MenuItem value={RmfTaskType.TYPE_DELIVERY}>Delivery</MenuItem>
            </TextField>
            <Grid container wrap="nowrap">
              <Grid style={{ flexGrow: 1 }}>
                <DateTimePicker
                  inputFormat={'MM/dd/yyyy HH:mm'}
                  value={new Date(task.start_time * 1000)}
                  onChange={(date) => {
                    if (!date) {
                      return;
                    }
                    // FIXME: needed because dateio typings default to moment
                    task.start_time = Math.floor((date as unknown as Date).getTime() / 1000);
                    updateTasks();
                  }}
                  label="Start Time"
                  renderInput={(props) => <TextField {...props} />}
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
      </StyledConfirmationDialog>
    </LocalizationProvider>
  );
}
