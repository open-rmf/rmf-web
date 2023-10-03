/**
 * FIXME(kp): Make the whole task request system task agnostic.
 * For that RMF needs to support task discovery and UI schemas https://github.com/open-rmf/rmf_api_msgs/issues/32.
 */

import UpdateIcon from '@mui/icons-material/Create';
import DeleteIcon from '@mui/icons-material/Delete';
import {
  Autocomplete,
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
  useTheme,
} from '@mui/material';
import { DatePicker, TimePicker, DateTimePicker } from '@mui/x-date-pickers';
import type { TaskFavoritePydantic as TaskFavorite, TaskRequest } from 'api-client';
import React from 'react';
import { Loading } from '..';
import { ConfirmationDialog, ConfirmationDialogProps } from '../confirmation-dialog';
import { PositiveIntField } from '../form-inputs';

// A bunch of manually defined descriptions to avoid using `any`.
interface Payload {
  sku: string;
  quantity: number;
}

interface DeliveryPlace {
  place: string;
  handler: string;
  payload: Payload;
}

interface DeliveryTaskDescription {
  pickup: DeliveryPlace;
  dropoff: DeliveryPlace;
}

interface PickupActivity {
  category: string;
  description: {
    pickup_zone: string;
    cart_rfid: number;
  };
}

interface DropoffActivity {
  category: string;
  description: {
    dropoff_point: string;
  };
}

interface DeliveryCustomTaskDescription {
  activities: [pickup_cart: PickupActivity, dropoff_cart: DropoffActivity];
}

type TaskDescription = DeliveryTaskDescription | DeliveryCustomTaskDescription;

const isNonEmptyString = (value: string): boolean => value.length > 0;
const isPositiveNumber = (value: number): boolean => value >= 0;

const isDeliveryPlaceValid = (place: DeliveryPlace, lookupMap: Record<string, string>): boolean => {
  return (
    isNonEmptyString(place.place) &&
    Object.keys(lookupMap).includes(place.place) &&
    isNonEmptyString(place.handler) &&
    Object.values(lookupMap).includes(place.handler) &&
    isNonEmptyString(place.payload.sku) &&
    isPositiveNumber(place.payload.quantity)
  );
};

const isDeliveryTaskDescriptionValid = (
  taskDescription: DeliveryTaskDescription,
  pickupPoints: Record<string, string>,
  dropoffPoints: Record<string, string>,
): boolean => {
  return (
    isDeliveryPlaceValid(taskDescription.pickup, pickupPoints) &&
    isDeliveryPlaceValid(taskDescription.dropoff, dropoffPoints)
  );
};

const isDeliveryCustomTaskDescriptionValid = (
  taskDescription: DeliveryCustomTaskDescription,
  pickupZones: string[],
  dropoffPoints: string[],
): boolean => {
  return (
    taskDescription.activities.length == 2 &&
    taskDescription.activities[0].category === 'perform_action' &&
    isNonEmptyString(taskDescription.activities[0].description.pickup_zone) &&
    pickupZones.includes(taskDescription.activities[0].description.pickup_zone) &&
    isPositiveNumber(taskDescription.activities[0].description.cart_rfid) &&
    taskDescription.activities[1].category === 'perform_action' &&
    isNonEmptyString(taskDescription.activities[1].description.dropoff_point) &&
    dropoffPoints.includes(taskDescription.activities[1].description.dropoff_point)
  );
};

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

export function getShortDescription(taskRequest: TaskRequest): string {
  switch (taskRequest.category) {
    case 'delivery': {
      return `[Delivery - 1:1] from [${taskRequest.description.pickup.place}] to [${taskRequest.description.dropoff.place}]`;
    }
    case 'delivery_sequential_lot_pick_up': {
      return `[Delivery - Sequential lot pick up] payload [${taskRequest.description.activities[0].cart_rfid}] from [${taskRequest.description.activities[0].pickup_zone}] to [${taskRequest.description.activities[1].dropoff_point}]`;
    }
    case 'delivery_area_pick_up': {
      return `[Delivery - Area pick up] payload [${taskRequest.description.activities[0].cart_rfid}] from [${taskRequest.description.activities[0].pickup_zone}] to [${taskRequest.description.activities[1].dropoff_point}]`;
    }
    default:
      return `[Unknown] type "${taskRequest.category}"`;
  }
}

interface DeliveryTaskFormProps {
  taskDesc: DeliveryTaskDescription;
  pickupPoints: Record<string, string>;
  dropoffPoints: Record<string, string>;
  onChange(taskDesc: TaskDescription): void;
  allowSubmit(allow: boolean): void;
}

function DeliveryTaskForm({
  taskDesc,
  pickupPoints = {},
  dropoffPoints = {},
  onChange,
  allowSubmit,
}: DeliveryTaskFormProps) {
  const theme = useTheme();
  const onInputChange = (desc: DeliveryTaskDescription) => {
    allowSubmit(isDeliveryTaskDescriptionValid(desc, pickupPoints, dropoffPoints));
    onChange(desc);
  };

  return (
    <Grid container spacing={theme.spacing(2)} justifyContent="left" alignItems="center">
      <Grid item xs={8}>
        <Autocomplete
          id="pickup-location"
          freeSolo
          fullWidth
          options={Object.keys(pickupPoints)}
          value={taskDesc.pickup.place}
          onInputChange={(_ev, newValue) => {
            const place = newValue ?? '';
            const handler =
              newValue !== null && pickupPoints[newValue] ? pickupPoints[newValue] : '';
            onInputChange({
              ...taskDesc,
              pickup: {
                ...taskDesc.pickup,
                place: place,
                handler: handler,
              },
            });
          }}
          onBlur={(ev) => {
            const handler = pickupPoints[(ev.target as HTMLInputElement).value] ?? '';
            onInputChange({
              ...taskDesc,
              pickup: {
                ...taskDesc.pickup,
                place: (ev.target as HTMLInputElement).value,
                handler: handler,
              },
            });
          }}
          renderInput={(params) => (
            <TextField
              {...params}
              label="Pickup Location"
              required
              error={!Object.keys(pickupPoints).includes(taskDesc.pickup.place)}
            />
          )}
        />
      </Grid>
      <Grid item xs={4}>
        <PositiveIntField
          id="pickup_sku"
          label="Cart ID"
          value={parseInt(taskDesc.pickup.payload.sku) ?? 0}
          onChange={(_ev, val) => {
            onInputChange({
              ...taskDesc,
              pickup: {
                ...taskDesc.pickup,
                payload: {
                  ...taskDesc.pickup.payload,
                  sku: val.toString(),
                },
              },
              dropoff: {
                ...taskDesc.dropoff,
                payload: {
                  ...taskDesc.dropoff.payload,
                  sku: val.toString(),
                },
              },
            });
          }}
          required
          error={
            Number.isNaN(parseInt(taskDesc.pickup.payload.sku)) ||
            Number.isNaN(parseInt(taskDesc.dropoff.payload.sku))
          }
        />
      </Grid>
      <Grid item xs={8}>
        <Autocomplete
          id="dropoff-location"
          freeSolo
          fullWidth
          options={Object.keys(dropoffPoints)}
          value={taskDesc.dropoff.place}
          onInputChange={(_ev, newValue) => {
            const place = newValue ?? '';
            const handler =
              newValue !== null && dropoffPoints[newValue] ? dropoffPoints[newValue] : '';
            onInputChange({
              ...taskDesc,
              dropoff: {
                ...taskDesc.dropoff,
                place: place,
                handler: handler,
              },
            });
          }}
          onBlur={(ev) =>
            dropoffPoints[(ev.target as HTMLInputElement).value] &&
            onInputChange({
              ...taskDesc,
              dropoff: {
                ...taskDesc.dropoff,
                place: (ev.target as HTMLInputElement).value,
                handler: dropoffPoints[(ev.target as HTMLInputElement).value],
              },
            })
          }
          renderInput={(params) => (
            <TextField
              {...params}
              label="Dropoff Location"
              required
              error={!Object.keys(dropoffPoints).includes(taskDesc.dropoff.place)}
            />
          )}
        />
      </Grid>
    </Grid>
  );
}

interface DeliveryCustomProps {
  taskDesc: DeliveryCustomTaskDescription;
  pickupZones: string[];
  cartIds: number[];
  dropoffPoints: string[];
  onChange(taskDesc: TaskDescription): void;
  allowSubmit(allow: boolean): void;
}

function DeliveryCustomTaskForm({
  taskDesc,
  pickupZones = [],
  cartIds = [],
  dropoffPoints = [],
  onChange,
  allowSubmit,
}: DeliveryCustomProps) {
  const theme = useTheme();
  const onInputChange = (desc: DeliveryCustomTaskDescription) => {
    allowSubmit(isDeliveryCustomTaskDescriptionValid(desc, pickupZones, dropoffPoints));
    onChange(desc);
  };

  return (
    <Grid container spacing={theme.spacing(2)} justifyContent="left" alignItems="center">
      <Grid item xs={8}>
        <Autocomplete
          id="pickup-zone"
          freeSolo
          fullWidth
          options={pickupZones}
          value={taskDesc.activities[0].description.pickup_zone}
          onInputChange={(_ev, newValue) => {
            if (newValue === null) {
              return;
            }
            const pickup_activity = taskDesc.activities[0];
            pickup_activity.description.pickup_zone = newValue;
            onInputChange({
              ...taskDesc,
              activities: [pickup_activity, taskDesc.activities[1]],
            });
          }}
          onBlur={(ev) => {
            const pickup_activity = taskDesc.activities[0];
            pickup_activity.description.pickup_zone = (ev.target as HTMLInputElement).value;
            onInputChange({
              ...taskDesc,
              activities: [pickup_activity, taskDesc.activities[1]],
            });
          }}
          renderInput={(params) => (
            <TextField
              {...params}
              label="Pickup Zone"
              required
              error={!pickupZones.includes(taskDesc.activities[0].description.pickup_zone)}
            />
          )}
        />
      </Grid>
      <Grid item xs={4}>
        <Autocomplete
          id="cart_rfid"
          freeSolo
          fullWidth
          options={cartIds.map(String)}
          value={String(taskDesc.activities[0].description.cart_rfid)}
          getOptionLabel={(option) => option}
          onInputChange={(_ev, newValue) => {
            if (newValue === null) {
              return;
            }
            const pickup_activity = taskDesc.activities[0];
            pickup_activity.description.cart_rfid = parseInt(newValue);
            onInputChange({
              ...taskDesc,
              activities: [pickup_activity, taskDesc.activities[1]],
            });
          }}
          onBlur={(ev) => {
            const pickup_activity = taskDesc.activities[0];
            pickup_activity.description.cart_rfid = parseInt((ev.target as HTMLInputElement).value);
            onInputChange({
              ...taskDesc,
              activities: [pickup_activity, taskDesc.activities[1]],
            });
          }}
          renderInput={(params) => (
            <TextField
              {...params}
              label="Cart ID"
              required
              error={Number.isNaN(taskDesc.activities[0].description.cart_rfid)}
            />
          )}
        />
      </Grid>
      <Grid item xs={8}>
        <Autocomplete
          id="dropoff-location"
          freeSolo
          fullWidth
          options={dropoffPoints}
          value={taskDesc.activities[1].description.dropoff_point}
          onInputChange={(_ev, newValue) => {
            if (newValue === null) {
              return;
            }
            const dropoff_activity = taskDesc.activities[1];
            dropoff_activity.description.dropoff_point = newValue;
            onInputChange({
              ...taskDesc,
              activities: [taskDesc.activities[0], dropoff_activity],
            });
          }}
          onBlur={(ev) => {
            const dropoff_activity = taskDesc.activities[1];
            dropoff_activity.description.dropoff_point = (ev.target as HTMLInputElement).value;
            onInputChange({
              ...taskDesc,
              activities: [taskDesc.activities[0], dropoff_activity],
            });
          }}
          renderInput={(params) => (
            <TextField
              {...params}
              label="Dropoff Location"
              required
              error={!dropoffPoints.includes(taskDesc.activities[1].description.dropoff_point)}
            />
          )}
        />
      </Grid>
    </Grid>
  );
}

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
        <ListItemText primary={listItemText} />
        <ListItemSecondaryAction>
          <IconButton
            edge="end"
            aria-label="update"
            onClick={() => {
              setCallToUpdate(true);
              listItemClick();
            }}
          >
            <UpdateIcon />
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
            <DeleteIcon />
          </IconButton>
        </ListItemSecondaryAction>
      </ListItem>
    </>
  );
}

function defaultDeliveryTask(): DeliveryTaskDescription {
  return {
    pickup: {
      place: '',
      handler: '',
      payload: {
        sku: '0',
        quantity: 1,
      },
    },
    dropoff: {
      place: '',
      handler: '',
      payload: {
        sku: '0',
        quantity: 1,
      },
    },
  };
}

function defaultDeliveryCustomTask(): DeliveryCustomTaskDescription {
  return {
    activities: [
      {
        category: 'perform_action',
        description: {
          pickup_zone: '',
          cart_rfid: 0,
        },
      },
      {
        category: 'perform_action',
        description: {
          dropoff_point: '',
        },
      },
    ],
  };
}

function defaultTaskDescription(taskCategory: string): TaskDescription | undefined {
  switch (taskCategory) {
    case 'delivery':
      return defaultDeliveryTask();
    case 'delivery_sequential_lot_pick_up':
    case 'delivery_area_pick_up':
      return defaultDeliveryCustomTask();
    default:
      return undefined;
  }
}

function defaultTask(): TaskRequest {
  return {
    category: 'delivery',
    description: defaultDeliveryTask(),
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
  const renderChip = (idx: number, text: string) => (
    <Chip
      key={idx}
      label={text}
      color="primary"
      sx={{ '&:hover': {}, margin: theme.spacing(0.25) }}
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
    category: 'delivery',
    description: defaultDeliveryTask(),
    unix_millis_earliest_start_time: 0,
    priority: { type: 'binary', value: 0 },
    user: '',
  };
};

export interface CreateTaskFormProps
  extends Omit<ConfirmationDialogProps, 'onConfirmClick' | 'toolbar'> {
  /**
   * Shows extra UI elements suitable for submittng batched tasks. Default to 'false'.
   */
  user: string;
  allowBatch?: boolean;
  cleaningZones?: string[];
  patrolWaypoints?: string[];
  pickupZones?: string[];
  cartIds?: number[];
  pickupPoints?: Record<string, string>;
  dropoffPoints?: Record<string, string>;
  favoritesTasks?: TaskFavorite[];
  scheduleToEdit?: Schedule;
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

  const [favoriteTaskBuffer, setFavoriteTaskBuffer] = React.useState<TaskFavorite>(
    defaultFavoriteTask(),
  );
  const [favoriteTaskTitleError, setFavoriteTaskTitleError] = React.useState(false);
  const [savingFavoriteTask, setSavingFavoriteTask] = React.useState(false);

  const [taskRequests, setTaskRequests] = React.useState<TaskRequest[]>(() => [
    requestTask ?? defaultTask(),
  ]);
  const [selectedTaskIdx, setSelectedTaskIdx] = React.useState(0);
  const taskTitles = React.useMemo(
    () => taskRequests && taskRequests.map((t, i) => `${i + 1}: ${getShortDescription(t)}`),
    [taskRequests],
  );
  const [submitting, setSubmitting] = React.useState(false);
  const [formFullyFilled, setFormFullyFilled] = React.useState(requestTask !== undefined || false);
  const taskRequest = taskRequests[selectedTaskIdx];
  const [openSchedulingDialog, setOpenSchedulingDialog] = React.useState(false);
  const [schedule, setSchedule] = React.useState<Schedule>(
    scheduleToEdit ?? {
      startOn: new Date(),
      days: [true, true, true, true, true, true, true],
      until: undefined,
      at: new Date(),
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
  // schedule is not supported with batch upload
  const scheduleEnabled = taskRequests.length === 1;

  const [warnTimeChecked, setWarnTimeChecked] = React.useState(false);
  const handleWarnTimeCheckboxChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setWarnTimeChecked(event.target.checked);
  };

  const updateTasks = () => {
    setTaskRequests((prev) => {
      prev.splice(selectedTaskIdx, 1, taskRequest);
      return [...prev];
    });
  };

  const handleTaskDescriptionChange = (newCategory: string, newDesc: TaskDescription) => {
    taskRequest.category = newCategory;
    taskRequest.description = newDesc;
    setFavoriteTaskBuffer({ ...favoriteTaskBuffer, description: newDesc, category: newCategory });
    updateTasks();
  };

  const renderTaskDescriptionForm = () => {
    switch (taskRequest.category) {
      case 'delivery':
        return (
          <DeliveryTaskForm
            taskDesc={taskRequest.description as DeliveryTaskDescription}
            pickupPoints={pickupPoints}
            dropoffPoints={dropoffPoints}
            onChange={(desc) => handleTaskDescriptionChange('delivery', desc)}
            allowSubmit={allowSubmit}
          />
        );
      case 'delivery_sequential_lot_pick_up':
      case 'delivery_area_pick_up':
        return (
          <DeliveryCustomTaskForm
            taskDesc={taskRequest.description as DeliveryCustomTaskDescription}
            pickupZones={pickupZones}
            cartIds={cartIds}
            dropoffPoints={Object.keys(dropoffPoints)}
            onChange={(desc) => handleTaskDescriptionChange(taskRequest.category, desc)}
            allowSubmit={allowSubmit}
          />
        );
      default:
        return null;
    }
  };
  const handleTaskTypeChange = (ev: React.ChangeEvent<HTMLInputElement>) => {
    const newCategory = ev.target.value;
    const newDesc = defaultTaskDescription(newCategory);
    if (newDesc === undefined) {
      return;
    }
    taskRequest.description = newDesc;
    taskRequest.category = newCategory;

    setFavoriteTaskBuffer({ ...favoriteTaskBuffer, category: newCategory, description: newDesc });

    updateTasks();
  };

  const allowSubmit = (allow: boolean) => {
    setFormFullyFilled(allow);
  };

  // no memo because deps would likely change
  const handleSubmit = async (scheduling: boolean) => {
    if (!submitTasks) {
      onSuccess && onSuccess(taskRequests);
      return;
    }

    const requester = scheduling ? `${user}__scheduled` : user;

    for (const t of taskRequests) {
      t.requester = requester;
      t.unix_millis_request_time = Date.now();
      console.log(t);
    }

    const submittingSchedule = scheduling && scheduleEnabled;
    try {
      setSubmitting(true);
      await submitTasks(taskRequests, submittingSchedule ? schedule : null);
      setSubmitting(false);

      if (submittingSchedule) {
        onSuccessScheduling && onSuccessScheduling();
      } else {
        onSuccess && onSuccess(taskRequests);
      }
    } catch (e) {
      setSubmitting(false);
      if (submittingSchedule) {
        onFailScheduling && onFailScheduling(e as Error);
      } else {
        onFail && onFail(e as Error, taskRequests);
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
      await submitFavoriteTask(favoriteTaskBuffer);
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

      setTaskRequests([defaultTask()]);
      setOpenFavoriteDialog(false);
      setCallToDeleteFavoriteTask(false);
      setCallToUpdateFavoriteTask(false);
    } catch (e) {
      setDeletingFavoriteTask(false);
      onFailFavoriteTask && onFailFavoriteTask(e as Error, favoriteTaskBuffer);
    }
  };

  /* eslint-disable @typescript-eslint/no-unused-vars */
  const handleSelectFileClick: React.MouseEventHandler<HTMLButtonElement> = () => {
    if (!tasksFromFile) {
      return;
    }
    (async () => {
      const newTasks = await tasksFromFile();
      if (newTasks.length === 0) {
        return;
      }
      setTaskRequests(newTasks);
      setSelectedTaskIdx(0);
    })();
  };

  const submitText = taskRequests.length > 1 ? 'Submit All Now' : 'Submit Now';

  return (
    <>
      <StyledDialog
        title="Create Task"
        maxWidth="lg"
        fullWidth={taskRequests.length > 1}
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
          <DialogContent>
            <Grid container direction="row" wrap="nowrap">
              <List dense className={classes.taskList} aria-label="Favorites Tasks">
                <Typography variant="h6" component="div">
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
                        setTaskRequests([
                          {
                            category: favoriteTask.category,
                            description: favoriteTask.description,
                            unix_millis_earliest_start_time: Date.now(),
                            priority: favoriteTask.priority,
                          },
                        ]);
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
                      value={taskRequest.category}
                      onChange={handleTaskTypeChange}
                    >
                      <MenuItem
                        value="delivery"
                        disabled={
                          Object.keys(pickupPoints).length === 0 ||
                          Object.keys(dropoffPoints).length === 0
                        }
                      >
                        Delivery - 1:1
                      </MenuItem>
                      <MenuItem
                        value="delivery_sequential_lot_pick_up"
                        disabled={Object.keys(dropoffPoints).length === 0}
                      >
                        Delivery - Sequential lot pick up
                      </MenuItem>
                      <MenuItem
                        value="delivery_area_pick_up"
                        disabled={Object.keys(dropoffPoints).length === 0}
                      >
                        Delivery - Area pick up
                      </MenuItem>
                    </TextField>
                  </Grid>
                  <Grid item xs={7}>
                    <DateTimePicker
                      inputFormat={'MM/dd/yyyy HH:mm'}
                      value={
                        taskRequest.unix_millis_earliest_start_time
                          ? new Date(taskRequest.unix_millis_earliest_start_time)
                          : new Date()
                      }
                      onChange={(date) => {
                        if (!date) {
                          return;
                        }
                        taskRequest.unix_millis_earliest_start_time = date.valueOf();
                        setFavoriteTaskBuffer({
                          ...favoriteTaskBuffer,
                          unix_millis_earliest_start_time: date.valueOf(),
                        });
                        updateTasks();
                      }}
                      label="Start Time"
                      renderInput={(props) => <TextField {...props} />}
                    />
                  </Grid>
                  <Grid item xs={1}>
                    <Checkbox
                      checked={warnTimeChecked}
                      onChange={handleWarnTimeCheckboxChange}
                      inputProps={{ 'aria-label': 'controlled' }}
                      sx={{ '& .MuiSvgIcon-root': { fontSize: 32 } }}
                    />
                  </Grid>
                  <Grid item xs={4}>
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
                        updateTasks();
                      }}
                      label="Warn Time"
                      renderInput={(props) => <TextField {...props} />}
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
                    onClick={() => {
                      !callToUpdateFavoriteTask &&
                        setFavoriteTaskBuffer({ ...favoriteTaskBuffer, name: '', id: '' });
                      setOpenFavoriteDialog(true);
                    }}
                    style={{ marginTop: theme.spacing(2), marginBottom: theme.spacing(2) }}
                  >
                    {callToUpdateFavoriteTask ? `Confirm edits` : 'Save as a favorite task'}
                  </Button>
                </Grid>
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
          </DialogContent>
          <DialogActions>
            <Button
              variant="outlined"
              disabled={submitting}
              className={classes.actionBtn}
              onClick={(ev) => onClose && onClose(ev, 'escapeKeyDown')}
            >
              Cancel
            </Button>
            <Button
              variant="contained"
              color="primary"
              disabled={submitting || !formFullyFilled}
              className={classes.actionBtn}
              onClick={() => setOpenSchedulingDialog(true)}
            >
              {scheduleToEdit ? 'Edit schedule' : 'Add to Schedule'}
            </Button>
            <Button
              variant="contained"
              type="submit"
              color="primary"
              disabled={submitting || !formFullyFilled || scheduleToEdit !== undefined}
              className={classes.actionBtn}
              aria-label={submitText}
              onClick={handleSubmitNow}
            >
              <Loading hideChildren loading={submitting} size="1.5em" color="inherit">
                {submitText}
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
                onChange={(date) =>
                  date &&
                  setSchedule((prev) => {
                    date.setHours(schedule.at.getHours());
                    date.setMinutes(schedule.at.getMinutes());
                    return { ...prev, startOn: date };
                  })
                }
                label="Start On"
                disabled={!scheduleEnabled}
                renderInput={(props) => <TextField {...props} fullWidth />}
              />
            </Grid>
            <Grid item xs={6}>
              <TimePicker
                value={schedule.at}
                onChange={(date) => {
                  if (!date) {
                    return;
                  }
                  setSchedule((prev) => ({ ...prev, at: date }));
                  if (!isNaN(date.valueOf())) {
                    setSchedule((prev) => {
                      const startOn = prev.startOn;
                      startOn.setHours(date.getHours());
                      startOn.setMinutes(date.getMinutes());
                      return { ...prev, startOn };
                    });
                  }
                }}
                label="At"
                disabled={!scheduleEnabled}
                renderInput={(props) => <TextField {...props} fullWidth />}
              />
            </Grid>
            <Grid item xs={12}>
              <DaySelectorSwitch
                value={schedule.days}
                disabled={!scheduleEnabled}
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
                    control={<Radio />}
                    label="Never"
                  />
                </Grid>
                <Grid item xs={2} paddingLeft={theme.spacing(1)}>
                  <FormControlLabel value={ScheduleUntilValue.ON} control={<Radio />} label="On" />
                </Grid>
                <Grid item xs={4}>
                  <DatePicker
                    value={
                      scheduleUntilValue === ScheduleUntilValue.NEVER ? new Date() : schedule.until
                    }
                    onChange={(date) =>
                      date &&
                      setSchedule((prev) => {
                        date.setHours(23);
                        date.setMinutes(59);
                        return { ...prev, until: date };
                      })
                    }
                    disabled={scheduleUntilValue !== ScheduleUntilValue.ON}
                    renderInput={(props) => <TextField {...props} fullWidth />}
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
