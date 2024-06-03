/**
 * FIXME(kp): Make the whole task request system task agnostic.
 * For that RMF needs to support task discovery and UI schemas https://github.com/open-rmf/rmf_api_msgs/issues/32.
 */

import UpdateIcon from '@mui/icons-material/Create';
import DeleteIcon from '@mui/icons-material/Delete';
import {
  Box,
  Button,
  Checkbox,
  Chip,
  Dialog,
  DialogActions,
  DialogContent,
  DialogProps,
  DialogTitle,
  Divider,
  FormControl,
  FormControlLabel,
  FormHelperText,
  Grid,
  IconButton,
  List,
  ListItem,
  ListItemSecondaryAction,
  ListItemText,
  MenuItem,
  Radio,
  RadioGroup,
  styled,
  TextField,
  Typography,
  useMediaQuery,
  useTheme,
} from '@mui/material';
import { DatePicker, TimePicker, DateTimePicker } from '@mui/x-date-pickers';
import type { TaskBookingLabel, TaskFavorite, TaskRequest } from 'api-client';
import React from 'react';
import { Loading } from '..';
import { ConfirmationDialog, ConfirmationDialogProps } from '../confirmation-dialog';
import {
  CleanTaskDescription,
  CleanTaskForm,
  DefaultCleanTaskDefinition,
  makeCleanTaskBookingLabel,
  makeDefaultCleanTaskDescription,
} from './types/clean';
import {
  DefaultCustomComposeTaskDefinition,
  CustomComposeTaskDescription,
  CustomComposeTaskForm,
  makeCustomComposeTaskBookingLabel,
} from './types/custom-compose';
import {
  DefaultSimpleDeliveryTaskDefinition,
  SimpleDeliveryTaskDescription,
  SimpleDeliveryTaskForm,
  makeDefaultSimpleDeliveryTaskDescription,
  makeSimpleDeliveryTaskBookingLabel,
} from './types/delivery-simple';
import {
  DefaultDeliveryPickupTaskDefinition,
  DefaultDeliveryAreaPickupTaskDefinition,
  DefaultDeliverySequentialLotPickupTaskDefinition,
  DeliveryCustomTaskForm,
  DeliveryCustomTaskDescription,
  DeliveryTaskDescription,
  DeliveryTaskForm,
  makeDefaultDeliveryCustomTaskDescription,
  makeDefaultDeliveryTaskDescription,
  makeDeliveryCustomTaskBookingLabel,
  makeDeliveryTaskBookingLabel,
} from './types/delivery-custom';
import {
  makeDefaultPatrolTask,
  makePatrolTaskBookingLabel,
  DefaultPatrolTaskDefinition,
  PatrolTaskDescription,
  PatrolTaskForm,
} from './types/patrol';
import { serializeTaskBookingLabel } from './task-booking-label-utils';

export interface TaskDefinition {
  task_definition_id: string;
  task_display_name: string;
}

// If no task definition id is found in a past task (scheduled or favorite)
const DefaultTaskDefinitionId = DefaultCustomComposeTaskDefinition.task_definition_id;

type TaskDescription =
  | DeliveryTaskDescription
  | DeliveryCustomTaskDescription
  | PatrolTaskDescription
  | SimpleDeliveryTaskDescription
  | CleanTaskDescription;

const classes = {
  title: 'dialogue-info-value',
  selectFileBtn: 'create-task-selected-file-btn',
  taskList: 'create-task-task-list',
  selectedTask: 'create-task-selected-task',
  actionBtn: 'dialogue-action-button',
};
const StyledDialog = styled((props: DialogProps) => <Dialog {...props} />)(({ theme }) => ({
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
  [`& .${classes.title}`]: {
    flex: '1 1 auto',
  },
  [`& .${classes.actionBtn}`]: {
    minWidth: 80,
  },
}));

interface FavoriteTaskProps {
  listItemText: string;
  listItemClick: () => void;
  favoriteTask: TaskFavorite;
  setFavoriteTask: (favoriteTask: TaskFavorite) => void;
  setOpenDialog: (open: boolean) => void;
  setCallToDelete: (open: boolean) => void;
  setCallToUpdate: (open: boolean) => void;
}

function FavoriteTask({
  listItemText,
  listItemClick,
  favoriteTask,
  setFavoriteTask,
  setOpenDialog,
  setCallToDelete,
  setCallToUpdate,
}: FavoriteTaskProps) {
  const theme = useTheme();
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  return (
    <>
      <ListItem
        sx={{ width: theme.spacing(30) }}
        onClick={() => {
          listItemClick();
          setCallToUpdate(false);
        }}
        role="listitem button"
        button
        divider={true}
      >
        <ListItemText
          primary={listItemText}
          primaryTypographyProps={{
            style: {
              fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
            },
          }}
        />
        <ListItemSecondaryAction>
          <IconButton
            edge="end"
            aria-label="update"
            onClick={() => {
              setCallToUpdate(true);
              listItemClick();
            }}
          >
            <UpdateIcon transform={`scale(${isScreenHeightLessThan800 ? 0.7 : 1})`} />
          </IconButton>
          <IconButton
            edge="end"
            aria-label="delete"
            onClick={() => {
              setOpenDialog(true);
              setFavoriteTask(favoriteTask);
              setCallToDelete(true);
            }}
          >
            <DeleteIcon transform={`scale(${isScreenHeightLessThan800 ? 0.7 : 1})`} />
          </IconButton>
        </ListItemSecondaryAction>
      </ListItem>
    </>
  );
}

function defaultTaskDescription(taskName: string): TaskDescription | undefined {
  switch (taskName) {
    case 'delivery_pickup':
      return makeDefaultDeliveryTaskDescription();
    case 'delivery_sequential_lot_pickup':
    case 'delivery_area_pickup':
      return makeDefaultDeliveryCustomTaskDescription(taskName);
    case 'patrol':
      return makeDefaultPatrolTask();
    case 'delivery':
      return makeDefaultSimpleDeliveryTaskDescription();
    case 'clean':
      return makeDefaultCleanTaskDescription();
    default:
      return undefined;
  }
}

function taskRequestCategory(task_definition_id: string): string | undefined {
  switch (task_definition_id) {
    case DefaultPatrolTaskDefinition.task_definition_id:
    case DefaultSimpleDeliveryTaskDefinition.task_definition_id:
      return task_definition_id;
    case DefaultDeliveryPickupTaskDefinition.task_definition_id:
    case DefaultDeliverySequentialLotPickupTaskDefinition.task_definition_id:
    case DefaultDeliveryAreaPickupTaskDefinition.task_definition_id:
    case DefaultCleanTaskDefinition.task_definition_id:
    case DefaultCustomComposeTaskDefinition.task_definition_id:
      return 'compose';
    default:
      return undefined;
  }
}

function defaultTaskRequest(task_definition_id: string): TaskRequest {
  const category = taskRequestCategory(task_definition_id);
  const description = defaultTaskDescription(task_definition_id);

  return {
    category: category ?? 'compose',
    description: description ?? makeDefaultDeliveryTaskDescription(),
    unix_millis_earliest_start_time: 0,
    unix_millis_request_time: Date.now(),
    priority: { type: 'binary', value: 0 },
    requester: '',
  };
}

export type RecurringDays = [boolean, boolean, boolean, boolean, boolean, boolean, boolean];

export interface Schedule {
  startOn: Date;
  days: RecurringDays;
  until?: Date;
  at: Date;
}

enum ScheduleUntilValue {
  NEVER = 'never',
  ON = 'on',
}

interface DaySelectorSwitchProps {
  disabled?: boolean;
  onChange: (checked: RecurringDays) => void;
  value: RecurringDays;
}

const DaySelectorSwitch: React.VFC<DaySelectorSwitchProps> = ({ disabled, onChange, value }) => {
  const theme = useTheme();
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  const renderChip = (idx: number, text: string) => (
    <Chip
      key={idx}
      label={text}
      color="primary"
      sx={{
        '&:hover': {},
        margin: theme.spacing(0.25),
        fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
      }}
      variant={value[idx] && !disabled ? 'filled' : 'outlined'}
      disabled={disabled}
      onClick={() => {
        value[idx] = !value[idx];
        onChange([...value]);
      }}
    />
  );
  return (
    <div>
      <TextField
        label="Recurring Every"
        color="primary"
        InputProps={{
          disabled: true,
          startAdornment: [
            renderChip(0, 'Mon'),
            renderChip(1, 'Tue'),
            renderChip(2, 'Wed'),
            renderChip(3, 'Thu'),
            renderChip(4, 'Fri'),
            renderChip(5, 'Sat'),
            renderChip(6, 'Sun'),
          ],
        }}
      />
    </div>
  );
};

const defaultFavoriteTask = (): TaskFavorite => {
  return {
    id: '',
    name: '',
    category: 'compose',
    description: makeDefaultDeliveryTaskDescription(),
    unix_millis_earliest_start_time: 0,
    priority: { type: 'binary', value: 0 },
    user: '',
    task_definition_id: DefaultTaskDefinitionId,
  };
};

export interface CreateTaskFormProps
  extends Omit<ConfirmationDialogProps, 'onConfirmClick' | 'toolbar'> {
  /**
   * Shows extra UI elements suitable for submittng batched tasks. Default to 'false'.
   */
  user: string;
  supportedTasks?: TaskDefinition[];
  allowBatch?: boolean;
  cleaningZones?: string[];
  patrolWaypoints?: string[];
  pickupZones?: string[];
  cartIds?: string[];
  pickupPoints?: Record<string, string>;
  dropoffPoints?: Record<string, string>;
  favoritesTasks?: TaskFavorite[];
  scheduleToEdit?: Schedule;
  // requestTask is provided only when editing a schedule
  requestTask?: TaskRequest;
  submitTasks?(tasks: TaskRequest[], schedule: Schedule | null): Promise<void>;
  tasksFromFile?(): Promise<TaskRequest[]> | TaskRequest[];
  onSuccess?(tasks: TaskRequest[]): void;
  onFail?(error: Error, tasks: TaskRequest[]): void;
  onSuccessFavoriteTask?(message: string, favoriteTask: TaskFavorite): void;
  onFailFavoriteTask?(error: Error, favoriteTask: TaskFavorite): void;
  submitFavoriteTask?(favoriteTask: TaskFavorite): Promise<void>;
  deleteFavoriteTask?(favoriteTask: TaskFavorite): Promise<void>;
  onSuccessScheduling?(): void;
  onFailScheduling?(error: Error): void;
}

export function CreateTaskForm({
  user,
  supportedTasks = [
    DefaultPatrolTaskDefinition,
    DefaultSimpleDeliveryTaskDefinition,
    DefaultCleanTaskDefinition,
    DefaultCustomComposeTaskDefinition,
  ],
  /* eslint-disable @typescript-eslint/no-unused-vars */
  cleaningZones = [],
  /* eslint-disable @typescript-eslint/no-unused-vars */
  patrolWaypoints = [],
  pickupZones = [],
  cartIds = [],
  pickupPoints = {},
  dropoffPoints = {},
  favoritesTasks = [],
  scheduleToEdit,
  requestTask,
  submitTasks,
  tasksFromFile,
  onClose,
  onSuccess,
  onFail,
  onSuccessFavoriteTask,
  onFailFavoriteTask,
  submitFavoriteTask,
  deleteFavoriteTask,
  onSuccessScheduling,
  onFailScheduling,
  ...otherProps
}: CreateTaskFormProps): JSX.Element {
  const theme = useTheme();

  const [openFavoriteDialog, setOpenFavoriteDialog] = React.useState(false);
  const [callToDeleteFavoriteTask, setCallToDeleteFavoriteTask] = React.useState(false);
  const [callToUpdateFavoriteTask, setCallToUpdateFavoriteTask] = React.useState(false);
  const [deletingFavoriteTask, setDeletingFavoriteTask] = React.useState(false);

  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  const [favoriteTaskBuffer, setFavoriteTaskBuffer] = React.useState<TaskFavorite>(
    defaultFavoriteTask(),
  );
  const [favoriteTaskTitleError, setFavoriteTaskTitleError] = React.useState(false);
  const [savingFavoriteTask, setSavingFavoriteTask] = React.useState(false);

  const [taskDefinitionId, setTaskDefinitionId] = React.useState<string>(
    supportedTasks.length > 0 ? supportedTasks[0].task_definition_id : DefaultTaskDefinitionId,
  );
  const [taskRequest, setTaskRequest] = React.useState<TaskRequest>(
    () =>
      requestTask ??
      defaultTaskRequest(
        supportedTasks.length > 0 ? supportedTasks[0].task_definition_id : DefaultTaskDefinitionId,
      ),
  );

  const [submitting, setSubmitting] = React.useState(false);
  const [formFullyFilled, setFormFullyFilled] = React.useState(requestTask !== undefined || false);
  const [openSchedulingDialog, setOpenSchedulingDialog] = React.useState(false);
  const defaultScheduleDate = new Date();
  defaultScheduleDate.setSeconds(0);
  defaultScheduleDate.setMilliseconds(0);

  const [schedule, setSchedule] = React.useState<Schedule>(
    scheduleToEdit ?? {
      startOn: defaultScheduleDate,
      days: [true, true, true, true, true, true, true],
      until: undefined,
      at: defaultScheduleDate,
    },
  );
  const [scheduleUntilValue, setScheduleUntilValue] = React.useState<string>(
    scheduleToEdit?.until ? ScheduleUntilValue.ON : ScheduleUntilValue.NEVER,
  );

  const handleScheduleUntilValue = (event: React.ChangeEvent<HTMLInputElement>) => {
    if (event.target.value === ScheduleUntilValue.ON) {
      /**
       * Since the time change is done in the onchange of the Datepicker,
       * we need a defined time if the user saves the value without calling the onChange event of the datepicker
       */
      const date = new Date();
      date.setHours(23);
      date.setMinutes(59);
      setSchedule((prev) => ({ ...prev, until: date }));
    } else {
      setSchedule((prev) => ({ ...prev, until: undefined }));
    }
    setScheduleUntilValue(event.target.value);
  };

  const [warnTimeChecked, setWarnTimeChecked] = React.useState(false);
  const handleWarnTimeCheckboxChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setWarnTimeChecked(event.target.checked);
  };

  const handleTaskDescriptionChange = (newCategory: string, newDesc: TaskDescription) => {
    setTaskRequest((prev) => {
      return {
        ...prev,
        category: newCategory,
        description: newDesc,
      };
    });
    setFavoriteTaskBuffer({ ...favoriteTaskBuffer, description: newDesc, category: newCategory });
  };

  // FIXME: Custom compose task descriptions are currently not allowed to be
  // saved as favorite tasks. This will probably require a re-write of
  // FavoriteTask's pydantic model with better typing.
  const handleCustomComposeTaskDescriptionChange = (newDesc: CustomComposeTaskDescription) => {
    setTaskRequest((prev) => {
      return {
        ...prev,
        category: 'custom_compose',
        description: newDesc,
      };
    });
  };

  const renderTaskDescriptionForm = () => {
    switch (taskRequest.category) {
      case 'patrol':
        return (
          <PatrolTaskForm
            taskDesc={taskRequest.description as PatrolTaskDescription}
            patrolWaypoints={patrolWaypoints}
            onChange={(desc) => handleTaskDescriptionChange('patrol', desc)}
            allowSubmit={allowSubmit}
          />
        );
      case 'delivery':
        return (
          <SimpleDeliveryTaskForm
            taskDesc={taskRequest.description as SimpleDeliveryTaskDescription}
            pickupPoints={pickupPoints}
            dropoffPoints={dropoffPoints}
            onChange={(desc) => handleTaskDescriptionChange('delivery', desc)}
            allowSubmit={allowSubmit}
          />
        );
      case 'custom_compose':
        return (
          <CustomComposeTaskForm
            taskDesc={taskRequest.description as CustomComposeTaskDescription}
            onChange={(desc) => {
              handleCustomComposeTaskDescriptionChange(desc);
            }}
            allowSubmit={allowSubmit}
          />
        );
    }

    switch (taskRequest.description.category) {
      case 'clean':
        return (
          <CleanTaskForm
            taskDesc={taskRequest.description as CleanTaskDescription}
            cleaningZones={cleaningZones}
            onChange={(desc: CleanTaskDescription) => {
              desc.category = taskRequest.description.category;
              handleTaskDescriptionChange('compose', desc);
            }}
            allowSubmit={allowSubmit}
          />
        );
      case 'delivery_pickup':
        return (
          <DeliveryTaskForm
            taskDesc={taskRequest.description as DeliveryTaskDescription}
            pickupPoints={pickupPoints}
            cartIds={cartIds}
            dropoffPoints={dropoffPoints}
            onChange={(desc: DeliveryTaskDescription) => {
              desc.category = taskRequest.description.category;
              desc.phases[0].activity.description.activities[1].description.category =
                taskRequest.description.category;
              handleTaskDescriptionChange('compose', desc);
              const pickupPerformAction =
                desc.phases[0].activity.description.activities[1].description.description;
            }}
            allowSubmit={allowSubmit}
          />
        );
      case 'delivery_sequential_lot_pickup':
      case 'delivery_area_pickup':
        return (
          <DeliveryCustomTaskForm
            taskDesc={taskRequest.description as DeliveryCustomTaskDescription}
            pickupZones={pickupZones}
            cartIds={cartIds}
            dropoffPoints={Object.keys(dropoffPoints)}
            onChange={(desc) => {
              desc.category = taskRequest.description.category;
              desc.phases[0].activity.description.activities[1].description.category =
                taskRequest.description.category;
              handleTaskDescriptionChange('compose', desc);
              const pickupPerformAction =
                desc.phases[0].activity.description.activities[1].description.description;
            }}
            allowSubmit={allowSubmit}
          />
        );
      default:
        return null;
    }
  };
  const handleTaskTypeChange = (ev: React.ChangeEvent<HTMLInputElement>) => {
    const newType = ev.target.value;
    setTaskDefinitionId(newType);

    if (newType === 'custom_compose') {
      taskRequest.category = 'custom_compose';
      taskRequest.description = '';
    } else {
      const newDesc = defaultTaskDescription(newType);
      if (newDesc === undefined) {
        return;
      }
      taskRequest.description = newDesc;

      const primaryTaskCategories = ['patrol', 'delivery'];
      const category = primaryTaskCategories.includes(newType) ? newType : 'compose';
      taskRequest.category = category;

      setFavoriteTaskBuffer({ ...favoriteTaskBuffer, category, description: newDesc });
    }
  };

  const allowSubmit = (allow: boolean) => {
    setFormFullyFilled(allow);
  };

  // no memo because deps would likely change
  const handleSubmit = async (scheduling: boolean) => {
    if (!submitTasks) {
      onSuccess && onSuccess([taskRequest]);
      return;
    }

    const requester = scheduling ? `${user}__scheduled` : user;

    const request = { ...taskRequest };
    request.requester = requester;
    request.unix_millis_request_time = Date.now();

    if (taskDefinitionId === 'custom_compose') {
      try {
        const obj = JSON.parse(request.description);
        request.category = 'compose';
        request.description = obj;
      } catch (e) {
        console.error('Invalid custom compose task description');
        onFail && onFail(e as Error, [request]);
        return;
      }
    }

    // Generate booking label for each task
    try {
      let requestBookingLabel: TaskBookingLabel | null = null;
      switch (taskDefinitionId) {
        case 'delivery_pickup':
          requestBookingLabel = makeDeliveryTaskBookingLabel(request.description);
          break;
        case 'delivery_sequential_lot_pickup':
        case 'delivery_area_pickup':
          requestBookingLabel = makeDeliveryCustomTaskBookingLabel(request.description);
          break;
        case 'patrol':
          requestBookingLabel = makePatrolTaskBookingLabel(request.description);
          break;
        case 'delivery':
          requestBookingLabel = makeSimpleDeliveryTaskBookingLabel(request.description);
        case 'clean':
          requestBookingLabel = makeCleanTaskBookingLabel(request.description);
        case 'custom_compose':
          requestBookingLabel = makeCustomComposeTaskBookingLabel();
          break;
      }

      if (!requestBookingLabel) {
        const error = Error(
          `Failed to generate booking label for task request of definition ID: ${taskDefinitionId}`,
        );
        onFail && onFail(error, [request]);
        return;
      }

      const labelString = serializeTaskBookingLabel(requestBookingLabel);
      if (labelString) {
        request.labels = [labelString];
      }
      console.log(`labels: ${request.labels}`);
    } catch (e) {
      console.error('Failed to generate string for task request label');
    }

    try {
      setSubmitting(true);
      await submitTasks([request], scheduling ? schedule : null);
      setSubmitting(false);

      if (scheduling) {
        onSuccessScheduling && onSuccessScheduling();
      } else {
        onSuccess && onSuccess([request]);
      }
    } catch (e) {
      setSubmitting(false);
      if (scheduling) {
        onFailScheduling && onFailScheduling(e as Error);
      } else {
        onFail && onFail(e as Error, [request]);
      }
    }
  };

  const handleSubmitNow: React.MouseEventHandler = async (ev) => {
    ev.preventDefault();
    await handleSubmit(false);
  };

  const handleSubmitSchedule: React.FormEventHandler = async (ev) => {
    ev.preventDefault();
    await handleSubmit(true);
  };

  const handleSubmitFavoriteTask: React.MouseEventHandler = async (ev) => {
    ev.preventDefault();

    if (!favoriteTaskBuffer.name) {
      setFavoriteTaskTitleError(true);
      return;
    }

    setFavoriteTaskTitleError(false);

    if (!submitFavoriteTask) {
      return;
    }
    try {
      setSavingFavoriteTask(true);

      const favoriteTask = favoriteTaskBuffer;
      favoriteTask.task_definition_id = taskDefinitionId ?? DefaultTaskDefinitionId;

      await submitFavoriteTask(favoriteTask);
      setSavingFavoriteTask(false);
      onSuccessFavoriteTask &&
        onSuccessFavoriteTask(
          `${!favoriteTaskBuffer.id ? `Created` : `Edited`}  favorite task successfully`,
          favoriteTaskBuffer,
        );
      setOpenFavoriteDialog(false);
      setCallToUpdateFavoriteTask(false);
    } catch (e) {
      setSavingFavoriteTask(false);
      onFailFavoriteTask && onFailFavoriteTask(e as Error, favoriteTaskBuffer);
    }
  };

  const handleDeleteFavoriteTask: React.MouseEventHandler = async (ev) => {
    ev.preventDefault();

    if (!deleteFavoriteTask) {
      return;
    }
    try {
      setDeletingFavoriteTask(true);
      await deleteFavoriteTask(favoriteTaskBuffer);
      setDeletingFavoriteTask(false);
      onSuccessFavoriteTask &&
        onSuccessFavoriteTask('Deleted favorite task successfully', favoriteTaskBuffer);

      setTaskRequest(
        supportedTasks && supportedTasks.length > 0
          ? defaultTaskRequest(supportedTasks[0].task_definition_id)
          : defaultTaskRequest('patrol'),
      );
      setOpenFavoriteDialog(false);
      setCallToDeleteFavoriteTask(false);
      setCallToUpdateFavoriteTask(false);
    } catch (e) {
      setDeletingFavoriteTask(false);
      onFailFavoriteTask && onFailFavoriteTask(e as Error, favoriteTaskBuffer);
    }
  };

  return (
    <>
      <StyledDialog
        title="Create Task"
        maxWidth={isScreenHeightLessThan800 ? 'md' : 'lg'}
        disableEnforceFocus
        {...otherProps}
      >
        <form aria-label="create-task">
          <DialogTitle>
            <Grid container wrap="nowrap" alignItems="center">
              <Grid item xs className={classes.title} container justifyContent="center">
                Create Task
              </Grid>
            </Grid>
          </DialogTitle>
          <DialogContent
            sx={{ padding: isScreenHeightLessThan800 ? '0px 1.5rem' : '1.25rem 1.5rem' }}
          >
            <Grid container direction="row" wrap="nowrap">
              <List dense className={classes.taskList} aria-label="Favorites Tasks">
                <Typography fontSize={isScreenHeightLessThan800 ? 16 : 20} component="div">
                  Favorite tasks
                </Typography>
                {favoritesTasks.map((favoriteTask, index) => {
                  return (
                    <FavoriteTask
                      listItemText={favoriteTask.name}
                      key={index}
                      setFavoriteTask={setFavoriteTaskBuffer}
                      favoriteTask={favoriteTask}
                      setCallToDelete={setCallToDeleteFavoriteTask}
                      setCallToUpdate={setCallToUpdateFavoriteTask}
                      setOpenDialog={setOpenFavoriteDialog}
                      listItemClick={() => {
                        setFavoriteTaskBuffer(favoriteTask);
                        setTaskRequest({
                          category: favoriteTask.category,
                          description: favoriteTask.description,
                          unix_millis_earliest_start_time: 0,
                          priority: favoriteTask.priority,
                        });
                        setTaskDefinitionId(favoriteTask.task_definition_id);
                      }}
                    />
                  );
                })}
              </List>

              <Divider
                orientation="vertical"
                flexItem
                style={{ marginLeft: theme.spacing(2), marginRight: theme.spacing(2) }}
              />

              <Grid>
                <Grid container spacing={theme.spacing(2)}>
                  <Grid item xs={12}>
                    <TextField
                      select
                      id="task-type"
                      label="Task Category"
                      variant="outlined"
                      fullWidth
                      margin="normal"
                      value={
                        taskRequest.category !== 'compose'
                          ? taskRequest.category
                          : taskRequest.description.category
                      }
                      onChange={handleTaskTypeChange}
                      sx={{
                        '& .MuiInputBase-input': {
                          fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
                          height: isScreenHeightLessThan800 ? '1.5rem' : '3.5rem',
                        },
                      }}
                      InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
                    >
                      {supportedTasks.map((taskDefinition) => {
                        return (
                          <MenuItem value={taskDefinition.task_definition_id}>
                            {taskDefinition.task_display_name}
                          </MenuItem>
                        );
                      })}
                    </TextField>
                  </Grid>
                  <Grid item xs={isScreenHeightLessThan800 ? 6 : 7}>
                    <DateTimePicker
                      inputFormat={'MM/dd/yyyy HH:mm'}
                      value={new Date()}
                      onChange={() => 0}
                      label="Start Time"
                      renderInput={(props) => (
                        <TextField
                          {...props}
                          InputLabelProps={{
                            style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 },
                          }}
                          sx={{
                            '& .MuiInputBase-input': {
                              fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
                            },
                          }}
                        />
                      )}
                      disabled
                    />
                  </Grid>
                  <Grid item xs={1}>
                    <Checkbox
                      checked={warnTimeChecked}
                      onChange={handleWarnTimeCheckboxChange}
                      inputProps={{ 'aria-label': 'controlled' }}
                      sx={{
                        '& .MuiSvgIcon-root': { fontSize: isScreenHeightLessThan800 ? 22 : 32 },
                      }}
                    />
                  </Grid>
                  <Grid item xs={isScreenHeightLessThan800 ? 5 : 4}>
                    <DateTimePicker
                      disabled={!warnTimeChecked}
                      inputFormat={'MM/dd/yyyy HH:mm'}
                      value={
                        taskRequest.unix_millis_warn_time
                          ? new Date(taskRequest.unix_millis_warn_time)
                          : new Date()
                      }
                      onChange={(date) => {
                        if (!date || !warnTimeChecked) {
                          return;
                        }
                        taskRequest.unix_millis_warn_time = date.valueOf();
                        setTaskRequest((prev) => {
                          return {
                            ...prev,
                            unix_millis_warn_time: date.valueOf(),
                          };
                        });
                      }}
                      label="Warn Time"
                      renderInput={(props) => (
                        <TextField
                          {...props}
                          InputLabelProps={{
                            style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 },
                          }}
                          sx={{
                            '& .MuiInputBase-input': {
                              fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
                            },
                          }}
                        />
                      )}
                    />
                  </Grid>
                </Grid>
                <Divider
                  orientation="horizontal"
                  flexItem
                  style={{ marginTop: theme.spacing(2), marginBottom: theme.spacing(2) }}
                />
                {renderTaskDescriptionForm()}
                <Grid container justifyContent="center">
                  <Button
                    aria-label="Save as a favorite task"
                    variant="contained"
                    color="primary"
                    size={isScreenHeightLessThan800 ? 'small' : 'medium'}
                    onClick={() => {
                      !callToUpdateFavoriteTask &&
                        setFavoriteTaskBuffer({ ...favoriteTaskBuffer, name: '', id: '' });
                      setOpenFavoriteDialog(true);
                    }}
                    style={{ marginTop: theme.spacing(2), marginBottom: theme.spacing(2) }}
                    disabled={taskDefinitionId === 'custom_compose'}
                  >
                    {callToUpdateFavoriteTask ? `Confirm edits` : 'Save as a favorite task'}
                  </Button>
                </Grid>
              </Grid>
            </Grid>
          </DialogContent>
          <DialogActions>
            <Button
              variant="outlined"
              disabled={submitting}
              className={classes.actionBtn}
              onClick={(ev) => onClose && onClose(ev, 'escapeKeyDown')}
              size={isScreenHeightLessThan800 ? 'small' : 'medium'}
            >
              Cancel
            </Button>
            <Button
              variant="contained"
              color="primary"
              disabled={submitting || !formFullyFilled}
              className={classes.actionBtn}
              onClick={() => setOpenSchedulingDialog(true)}
              size={isScreenHeightLessThan800 ? 'small' : 'medium'}
            >
              {scheduleToEdit ? 'Edit schedule' : 'Add to Schedule'}
            </Button>
            <Button
              variant="contained"
              type="submit"
              color="primary"
              disabled={submitting || !formFullyFilled || scheduleToEdit !== undefined}
              className={classes.actionBtn}
              aria-label="Submit Now"
              onClick={handleSubmitNow}
              size={isScreenHeightLessThan800 ? 'small' : 'medium'}
            >
              <Loading
                hideChildren
                loading={submitting}
                size={isScreenHeightLessThan800 ? '0.8em' : '1.5em'}
                color="inherit"
              >
                Submit Now
              </Loading>
            </Button>
          </DialogActions>
        </form>
      </StyledDialog>
      {openFavoriteDialog && (
        <ConfirmationDialog
          confirmText={callToDeleteFavoriteTask ? 'Delete' : 'Save'}
          cancelText="Back"
          open={openFavoriteDialog}
          title={callToDeleteFavoriteTask ? 'Confirm Delete' : 'Favorite Task'}
          submitting={callToDeleteFavoriteTask ? deletingFavoriteTask : savingFavoriteTask}
          onClose={() => {
            setOpenFavoriteDialog(false);
            setCallToDeleteFavoriteTask(false);
          }}
          onSubmit={callToDeleteFavoriteTask ? handleDeleteFavoriteTask : handleSubmitFavoriteTask}
        >
          {!callToDeleteFavoriteTask && (
            <TextField
              size="small"
              value={favoriteTaskBuffer.name}
              onChange={(e) =>
                setFavoriteTaskBuffer({ ...favoriteTaskBuffer, name: e.target.value })
              }
              helperText="Required"
              error={favoriteTaskTitleError}
            />
          )}
          {callToDeleteFavoriteTask && (
            <Typography>{`Are you sure you want to delete "${favoriteTaskBuffer.name}"?`}</Typography>
          )}
        </ConfirmationDialog>
      )}
      {openSchedulingDialog && (
        <ConfirmationDialog
          confirmText="Schedule"
          cancelText="Cancel"
          open={openSchedulingDialog}
          title="Schedule Task"
          submitting={false}
          onClose={() => setOpenSchedulingDialog(false)}
          onSubmit={(ev) => {
            handleSubmitSchedule(ev);
            setOpenSchedulingDialog(false);
          }}
        >
          <Grid container spacing={theme.spacing(2)} marginTop={theme.spacing(1)}>
            <Grid item xs={6}>
              <DatePicker
                value={schedule.startOn}
                onChange={(date) => {
                  if (!date) {
                    console.error('DatePicker: invalid date');
                    return;
                  }
                  console.debug(`DatePicker: ${date}`);
                  setSchedule((prev) => {
                    date.setHours(schedule.at.getHours());
                    date.setMinutes(schedule.at.getMinutes());
                    date.setSeconds(0);
                    date.setMilliseconds(0);
                    console.debug(`DatePicker setSchedule: ${date}`);
                    return { ...prev, startOn: date };
                  });
                }}
                label="Start On"
                renderInput={(props) => (
                  <TextField
                    {...props}
                    size="small"
                    fullWidth
                    sx={{
                      '& .MuiInputBase-input': {
                        fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
                      },
                    }}
                  />
                )}
              />
            </Grid>
            <Grid item xs={6}>
              <TimePicker
                value={schedule.at}
                onChange={(date) => {
                  if (!date) {
                    console.error('TimePicker: invalid date');
                    return;
                  }
                  console.debug(`TimePicker: ${date}`);
                  if (!isNaN(date.valueOf())) {
                    setSchedule((prev) => {
                      const startOn = prev.startOn;
                      startOn.setHours(date.getHours());
                      startOn.setMinutes(date.getMinutes());
                      startOn.setSeconds(0);
                      startOn.setMilliseconds(0);
                      console.debug(`TimePicker setSchedule: ${date}`);
                      return { ...prev, at: date, startOn };
                    });
                  }
                }}
                label="At"
                renderInput={(props) => (
                  <TextField
                    {...props}
                    fullWidth
                    size="small"
                    sx={{
                      '& .MuiInputBase-input': {
                        fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
                      },
                    }}
                  />
                )}
              />
            </Grid>
            <Grid item xs={12}>
              <DaySelectorSwitch
                value={schedule.days}
                onChange={(days) => setSchedule((prev) => ({ ...prev, days }))}
              />
            </Grid>
          </Grid>
          <Grid container marginTop={theme.spacing(1)} marginLeft={theme.spacing(0)}>
            <FormControl fullWidth={true}>
              <FormHelperText>Ends</FormHelperText>
              <RadioGroup
                aria-labelledby="controlled-radio-buttons-group"
                name="controlled-radio-buttons-group"
                value={scheduleUntilValue}
                onChange={handleScheduleUntilValue}
                row
              >
                <Grid item xs={6} paddingLeft={theme.spacing(1)}>
                  <FormControlLabel
                    value={ScheduleUntilValue.NEVER}
                    control={<Radio size={isScreenHeightLessThan800 ? 'small' : 'medium'} />}
                    label={
                      <Box component="div" fontSize={isScreenHeightLessThan800 ? '0.8rem' : '1rem'}>
                        Never
                      </Box>
                    }
                    sx={{ fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem' }}
                  />
                </Grid>
                <Grid item xs={2} paddingLeft={theme.spacing(1)}>
                  <FormControlLabel
                    value={ScheduleUntilValue.ON}
                    control={<Radio size={isScreenHeightLessThan800 ? 'small' : 'medium'} />}
                    label={
                      <Box component="div" fontSize={isScreenHeightLessThan800 ? '0.8rem' : '1rem'}>
                        On
                      </Box>
                    }
                  />
                </Grid>
                <Grid item xs={4}>
                  <DatePicker
                    value={
                      scheduleUntilValue === ScheduleUntilValue.NEVER ? new Date() : schedule.until
                    }
                    onChange={(date) => {
                      if (!date) {
                        console.error('Until DatePicker: invalid date');
                        return;
                      }
                      console.debug(`Until DatePicker: ${date}`);
                      setSchedule((prev) => {
                        date.setHours(23);
                        date.setMinutes(59);
                        console.debug(`Until DatePicker setSchedule: ${date}`);
                        return { ...prev, until: date };
                      });
                    }}
                    disabled={scheduleUntilValue !== ScheduleUntilValue.ON}
                    renderInput={(props) => (
                      <TextField
                        {...props}
                        fullWidth
                        sx={{
                          '& .MuiInputBase-input': {
                            fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
                          },
                        }}
                      />
                    )}
                  />
                </Grid>
              </RadioGroup>
            </FormControl>
          </Grid>
        </ConfirmationDialog>
      )}
    </>
  );
}
