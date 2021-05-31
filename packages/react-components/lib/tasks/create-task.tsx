import DateFnsUtils from '@date-io/date-fns';
import {
  Button,
  CircularProgress,
  Dialog,
  DialogActions,
  DialogContent,
  DialogProps,
  DialogTitle,
  Divider,
  Grid,
  makeStyles,
  MenuItem,
  TextField,
  Typography,
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

type TaskDescription = CleanTaskDescription | LoopTaskDescription | DeliveryTaskDescription;

const useStyles = makeStyles((theme) => ({
  uploadFileBtn: {
    marginBottom: theme.spacing(1),
  },
}));

interface FormToolbarProps {
  batchMode: boolean;
  onUploadFileClick?(ev: React.MouseEvent<HTMLButtonElement>): void;
}

function FormToolbar({ batchMode, onUploadFileClick }: FormToolbarProps) {
  const classes = useStyles();

  return (
    <Grid container wrap="nowrap" alignItems="center">
      <Grid style={{ flexGrow: 1 }}>
        <Typography variant="h6">Create Task</Typography>
      </Grid>
      {batchMode && (
        <Grid>
          <Button
            aria-label="Upload File"
            className={classes.uploadFileBtn}
            variant="contained"
            color="primary"
            onClick={onUploadFileClick}
          >
            Upload File
          </Button>
        </Grid>
      )}
    </Grid>
  );
}

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
        <Grid style={{ flex: '1 1 60%' }}>
          <Autocomplete
            id="pickup-location"
            freeSolo
            autoSelect
            fullWidth
            options={deliveryWaypoints}
            onChange={(_ev, newValue) =>
              onChange({
                ...taskDesc,
                pickup_place_name: newValue,
              })
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
            autoSelect
            fullWidth
            options={dispensers}
            onChange={(_ev, newValue) =>
              onChange({
                ...taskDesc,
                pickup_dispenser: newValue,
              })
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
            autoSelect
            fullWidth
            options={deliveryWaypoints}
            onChange={(_ev, newValue) =>
              onChange({
                ...taskDesc,
                dropoff_place_name: newValue,
              })
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
            autoSelect
            fullWidth
            options={ingestors}
            onChange={(_ev, newValue) =>
              onChange({
                ...taskDesc,
                dropoff_ingestor: newValue,
              })
            }
            renderInput={(params) => <TextField {...params} label="Ingestor" margin="normal" />}
          />
        </Grid>
      </Grid>
    </>
  );
}

interface LoopTaskFormProps {
  taskDesc: LoopTaskDescription;
  loopWaypoints: string[];
  onChange(loopTaskDescription: LoopTaskDescription): void;
}

function LoopTaskForm({ taskDesc, loopWaypoints, onChange }: LoopTaskFormProps) {
  const theme = useTheme();
  const [numOfLoops, setNumOfLoops] = React.useState(taskDesc.num_loops.toString());

  return (
    <>
      <Autocomplete
        id="start-location"
        freeSolo
        autoSelect
        fullWidth
        options={loopWaypoints}
        onChange={(_ev, newValue) =>
          onChange({
            ...taskDesc,
            start_name: newValue,
          })
        }
        renderInput={(params) => <TextField {...params} label="Start Location" margin="normal" />}
      />
      <Grid container wrap="nowrap">
        <Grid style={{ flex: '1 1 100%' }}>
          <Autocomplete
            id="finish-location"
            freeSolo
            autoSelect
            fullWidth
            options={loopWaypoints}
            onChange={(_ev, newValue) =>
              onChange({
                ...taskDesc,
                finish_name: newValue,
              })
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
          <TextField
            id="loops"
            type="number"
            label="Loops"
            margin="normal"
            value={numOfLoops}
            onChange={(ev) => {
              setNumOfLoops(ev.target.value);
              onChange({
                ...taskDesc,
                num_loops: parseInt(ev.target.value) || 1,
              });
            }}
          />
        </Grid>
      </Grid>
    </>
  );
}

interface CleanTaskFormProps {
  taskDesc: CleanTaskDescription;
  cleaningZones: string[];
  onChange(cleanTaskDescription: CleanTaskDescription): void;
}

function CleanTaskForm({ taskDesc, cleaningZones, onChange }: CleanTaskFormProps) {
  return (
    <Autocomplete
      id="cleaning-zone"
      freeSolo
      autoSelect
      fullWidth
      options={cleaningZones}
      onChange={(_ev, newValue) =>
        onChange({
          ...taskDesc,
          cleaning_zone: newValue,
        })
      }
      renderInput={(params) => <TextField {...params} label="Cleaning Zone" margin="normal" />}
    />
  );
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

export interface CreateTaskFormProps extends DialogProps {
  /**
   * Shows extra UI elements suitable for submittng batched tasks. Default to 'false'.
   */
  batchMode?: boolean;
  cleaningZones?: string[];
  loopWaypoints?: string[];
  deliveryWaypoints?: string[];
  dispensers?: string[];
  ingestors?: string[];
  submitTask?(task: SubmitTask): Promise<void>;
  onSuccess?(task: SubmitTask): void;
  onFail?(error: Error, task: SubmitTask): void;
  onCancelClick?(ev: React.MouseEvent<HTMLButtonElement>): void;
  onUploadFileClick?(ev: React.MouseEvent<HTMLButtonElement>): void;
}

export function CreateTaskForm({
  batchMode = false,
  cleaningZones = [],
  loopWaypoints = [],
  deliveryWaypoints = [],
  dispensers = [],
  ingestors = [],
  submitTask,
  onSuccess,
  onFail,
  onCancelClick,
  onUploadFileClick,
  ...dialogProps
}: CreateTaskFormProps): JSX.Element {
  const theme = useTheme();
  const [taskType, setTaskType] = React.useState<number | undefined>(undefined);
  const [startDate, setStartDate] = React.useState(new Date());
  const [priority, setPriority] = React.useState(0);
  const [priorityInput, setPriorityInput] = React.useState('0');
  const [taskDescription, setTaskDescrption] = React.useState<TaskDescription | undefined>(() =>
    defaultTaskDescription(taskType),
  );
  const [submitting, setSubmitting] = React.useState(false);

  const renderTaskDescriptionForm = () => {
    if (!taskType || !taskDescription) {
      return null;
    }
    switch (taskType) {
      case RmfModels.TaskType.TYPE_CLEAN:
        return (
          <CleanTaskForm
            taskDesc={taskDescription as CleanTaskDescription}
            cleaningZones={cleaningZones}
            onChange={setTaskDescrption}
          />
        );
      case RmfModels.TaskType.TYPE_LOOP:
        return (
          <LoopTaskForm
            taskDesc={taskDescription as LoopTaskDescription}
            loopWaypoints={loopWaypoints}
            onChange={setTaskDescrption}
          />
        );
      case RmfModels.TaskType.TYPE_DELIVERY:
        return (
          <DeliveryTaskForm
            taskDesc={taskDescription as DeliveryTaskDescription}
            deliveryWaypoints={deliveryWaypoints}
            dispensers={dispensers}
            ingestors={ingestors}
            onChange={setTaskDescrption}
          />
        );
      default:
        return null;
    }
  };

  const handleTaskTypeChange = React.useCallback((ev: React.ChangeEvent<HTMLInputElement>) => {
    const newType = parseInt(ev.target.value);
    setTaskDescrption(defaultTaskDescription(newType));
    setTaskType(newType);
  }, []);

  // no memo because deps would likely change
  const handleSubmit: React.MouseEventHandler<HTMLButtonElement> = (ev) => {
    ev.preventDefault();
    (async () => {
      const task: SubmitTask = {
        task_type: taskType,
        start_time: Math.floor(startDate.valueOf() / 1000),
        priority,
        description: taskDescription,
      };
      if (!submitTask) {
        onSuccess && onSuccess(task);
        return;
      }
      try {
        setSubmitting(true);
        await submitTask(task);
        onSuccess && onSuccess(task);
      } catch (e) {
        onFail && onFail(e, task);
      } finally {
        setSubmitting(false);
      }
    })();
  };

  return (
    <MuiPickersUtilsProvider utils={DateFnsUtils}>
      <Dialog {...dialogProps}>
        <form>
          <DialogTitle>
            <FormToolbar batchMode={batchMode} onUploadFileClick={onUploadFileClick} />
          </DialogTitle>
          <Divider />
          <DialogContent>
            <TextField
              select
              id="task-type"
              label="Task Type"
              variant="outlined"
              fullWidth
              margin="normal"
              value={taskType || ''}
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
                  id="priority"
                  type="number"
                  label="Priority"
                  margin="normal"
                  value={priorityInput}
                  onChange={(ev) => setPriorityInput(ev.target.value)}
                  onBlur={() => {
                    const newPriority = parseInt(priorityInput) || 0;
                    setPriority(newPriority);
                    setPriorityInput(newPriority.toString());
                  }}
                />
              </Grid>
            </Grid>
            {renderTaskDescriptionForm()}
          </DialogContent>
          <Divider />
          <DialogActions>
            <Button
              variant="contained"
              color="primary"
              disabled={submitting}
              onClick={onCancelClick}
              aria-label="Cancel"
            >
              Cancel
            </Button>
            <Button
              style={{ margin: theme.spacing(1) }}
              type="submit"
              variant="contained"
              color="primary"
              disabled={submitting}
              onClick={handleSubmit}
              aria-label="Submit"
            >
              <Typography
                style={{ visibility: submitting ? 'hidden' : 'visible' }}
                variant="button"
              >
                Submit
              </Typography>
              <CircularProgress
                style={{ position: 'absolute', visibility: submitting ? 'visible' : 'hidden' }}
                color="inherit"
                size="1.8em"
              />
            </Button>
          </DialogActions>
        </form>
      </Dialog>
    </MuiPickersUtilsProvider>
  );
}
