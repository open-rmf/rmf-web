/**
 * FIXME(kp): Make the whole task request system task agnostic.
 * For that RMF needs to support task discovery and UI schemas https://github.com/open-rmf/rmf_api_msgs/issues/32.
 */

import UpdateIcon from '@mui/icons-material/Create';
import DeleteIcon from '@mui/icons-material/Delete';
import FavoriteBorder from '@mui/icons-material/FavoriteBorder';
import SaveIcon from '@mui/icons-material/Save';
import ScheduleSendIcon from '@mui/icons-material/ScheduleSend';
import SendIcon from '@mui/icons-material/Send';
import {
  Box,
  Button,
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
  Switch,
  TextField,
  Tooltip,
  Typography,
  useMediaQuery,
  useTheme,
} from '@mui/material';
import { DatePicker, DateTimePicker } from '@mui/x-date-pickers';
import { TimePicker } from '@mui/x-date-pickers/TimePicker';
import type { TaskFavorite, TaskRequest } from 'api-client';
import React from 'react';

import { Loading } from '..';
import { ConfirmationDialog, ConfirmationDialogProps } from '../confirmation-dialog';
import { TaskBookingLabels } from './booking-label';
import {
  getTaskBookingLabelFromTaskRequest,
  getTaskDefinitionId,
  serializeTaskBookingLabel,
} from './task-booking-label-utils';
import {
  ComposeCleanTaskDefinition,
  ComposeCleanTaskDescription,
  ComposeCleanTaskForm,
  makeComposeCleanTaskBookingLabel,
} from './types/compose-clean';
import {
  CustomComposeTaskDefinition,
  CustomComposeTaskDescription,
  CustomComposeTaskForm,
  makeCustomComposeTaskBookingLabel,
} from './types/custom-compose';
import {
  DeliveryTaskDefinition,
  DeliveryTaskDescription,
  DeliveryTaskForm,
  makeDeliveryTaskBookingLabel,
} from './types/delivery';
import {
  DeliveryAreaPickupTaskDefinition,
  DeliveryCustomTaskDescription,
  DeliveryCustomTaskForm,
  DeliveryPickupTaskDefinition,
  DeliveryPickupTaskDescription,
  DeliveryPickupTaskForm,
  DeliverySequentialLotPickupTaskDefinition,
  makeDeliveryCustomTaskBookingLabel,
  makeDeliveryPickupTaskBookingLabel,
} from './types/delivery-custom';
import {
  makePatrolTaskBookingLabel,
  PatrolTaskDefinition,
  PatrolTaskDescription,
  PatrolTaskForm,
} from './types/patrol';
import { getDefaultTaskDescription, getTaskRequestCategory } from './types/utils';
import { createTaskPriority, parseTaskPriority } from './utils';

export interface TaskDefinition {
  taskDefinitionId: string;
  taskDisplayName: string;
  requestCategory: string;
  scheduleEventColor?: string;
}

export type TaskDescription =
  | DeliveryPickupTaskDescription
  | DeliveryCustomTaskDescription
  | PatrolTaskDescription
  | DeliveryTaskDescription
  | ComposeCleanTaskDescription;

const classes = {
  title: 'dialogue-info-value',
  taskList: 'create-task-task-list',
  actionBtn: 'dialogue-action-button',
};
const StyledDialog = styled((props: DialogProps) => <Dialog {...props} />)(() => ({
  [`& .${classes.taskList}`]: {
    flex: '1 1 auto',
    minHeight: 400,
    maxHeight: '50vh',
    overflow: 'auto',
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

function getDefaultTaskRequest(taskDefinitionId: string): TaskRequest | null {
  const category = getTaskRequestCategory(taskDefinitionId);
  const description = getDefaultTaskDescription(taskDefinitionId);

  if (category === undefined) {
    console.error(`Unable to retrieve task category for task definition of id ${taskDefinitionId}`);
  }
  if (description === undefined) {
    console.error(
      `Unable to retrieve task description for task definition of id ${taskDefinitionId}`,
    );
  }

  if (category !== undefined && description !== undefined) {
    return {
      category,
      description,
      unix_millis_earliest_start_time: 0,
      unix_millis_request_time: Date.now(),
      priority: createTaskPriority(false),
      requester: '',
    };
  }

  return null;
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

export interface RobotDispatchTarget {
  fleet: string;
  robot: string;
}

export interface TaskFormProps extends Omit<ConfirmationDialogProps, 'onConfirmClick' | 'toolbar'> {
  user: string;
  fleets?: Record<string, string[]>;
  tasksToDisplay?: TaskDefinition[];
  cleaningZones?: string[];
  patrolWaypoints?: string[];
  pickupZones?: string[];
  cartIds?: string[];
  pickupPoints?: Record<string, string>;
  dropoffPoints?: Record<string, string>;
  favoritesTasks?: TaskFavorite[];
  schedule?: Schedule;
  taskRequest?: TaskRequest;
  onDispatchTask?(
    task: TaskRequest,
    robotDispatchTarget: RobotDispatchTarget | null,
  ): Promise<void>;
  /** If provided, the button Schedule Task will be rendered and clicking it will call this callback */
  onScheduleTask?(task: TaskRequest, schedule: Schedule): Promise<void>;
  /** If provided, the button Edit Schedule will be rendered and clicking it will call this callback */
  onEditScheduleTask?(task: TaskRequest, schedule: Schedule): Promise<void>;
  onSuccess?(task: TaskRequest): void;
  onFail?(error: Error, task?: TaskRequest): void;
  onSuccessFavoriteTask?(message: string, favoriteTask: TaskFavorite): void;
  onFailFavoriteTask?(error: Error, favoriteTask: TaskFavorite): void;
  submitFavoriteTask?(favoriteTask: TaskFavorite): Promise<void>;
  deleteFavoriteTask?(favoriteTask: TaskFavorite): Promise<void>;
  onSuccessScheduling?(): void;
  onFailScheduling?(error: Error): void;
}

export function TaskForm({
  user,
  fleets,
  tasksToDisplay = [
    PatrolTaskDefinition,
    DeliveryTaskDefinition,
    ComposeCleanTaskDefinition,
    CustomComposeTaskDefinition,
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
  schedule,
  taskRequest,
  onDispatchTask,
  onScheduleTask,
  onEditScheduleTask,
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
}: TaskFormProps): JSX.Element {
  const theme = useTheme();

  const [openFavoriteDialog, setOpenFavoriteDialog] = React.useState(false);
  const [callToDeleteFavoriteTask, setCallToDeleteFavoriteTask] = React.useState(false);
  const [callToUpdateFavoriteTask, setCallToUpdateFavoriteTask] = React.useState(false);
  const [deletingFavoriteTask, setDeletingFavoriteTask] = React.useState(false);

  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  // Note that we are not checking if the number of supported tasks is larger
  // than 0, this will cause the dashboard to fail when the create task form is
  // opened. This is intentional as it is a misconfiguration and will require
  // the build-time configuration to be fixed.
  const { defaultTaskDescription, defaultTaskRequest, validTasks } = React.useMemo(() => {
    let defaultTaskDescription: string | TaskDescription | null = null;
    let defaultTaskRequest: TaskRequest | null = null;
    const validTasks: TaskDefinition[] = [];

    tasksToDisplay.forEach((supportedTask: TaskDefinition) => {
      const definitionId = supportedTask.taskDefinitionId;
      const desc = getDefaultTaskDescription(definitionId);
      const req = getDefaultTaskRequest(definitionId);

      if (desc === undefined) {
        console.error(`Failed to retrieve task description for definition ID: [${definitionId}]`);
      }
      if (req === null) {
        console.error(`Failed to create task request for definition ID: [${definitionId}]`);
      }
      if (desc !== undefined && req !== null) {
        validTasks.push(supportedTask);

        if (!defaultTaskDescription && !defaultTaskRequest) {
          defaultTaskDescription = desc;
          defaultTaskRequest = req;
        }
      }
    });
    return { defaultTaskDescription, defaultTaskRequest, validTasks };
  }, [tasksToDisplay]);

  if (!defaultTaskDescription || !defaultTaskRequest) {
    // We should never reach this state unless a misconfiguration happened.
    const err = Error('Default task could not be generated, this might be a configuration error');
    onFail && onFail(err);
    console.error(err.message);
    throw new TypeError(err.message);
  }

  const [favoriteTaskBuffer, setFavoriteTaskBuffer] = React.useState<TaskFavorite>({
    id: '',
    name: '',
    category: tasksToDisplay[0].requestCategory,
    description: defaultTaskDescription,
    unix_millis_earliest_start_time: 0,
    priority: { type: 'binary', value: 0 },
    user: '',
    task_definition_id: tasksToDisplay[0].taskDefinitionId,
  });
  const [favoriteTaskTitleError, setFavoriteTaskTitleError] = React.useState(false);
  const [savingFavoriteTask, setSavingFavoriteTask] = React.useState(false);

  const [currentTaskRequest, setCurrentTaskRequest] = React.useState<TaskRequest>(
    taskRequest ?? defaultTaskRequest,
  );
  const initialBookingLabel = taskRequest ? getTaskBookingLabelFromTaskRequest(taskRequest) : null;
  const [taskDefinitionId, setTaskDefinitionId] = React.useState<string>(() => {
    const fromLabel = initialBookingLabel && getTaskDefinitionId(initialBookingLabel);
    return fromLabel || tasksToDisplay[0].taskDefinitionId;
  });

  const [submitting, setSubmitting] = React.useState(false);
  const [formFullyFilled, setFormFullyFilled] = React.useState(taskRequest !== undefined || false);
  const [openSchedulingDialog, setOpenSchedulingDialog] = React.useState(false);
  const defaultScheduleDate = new Date();
  defaultScheduleDate.setSeconds(0);
  defaultScheduleDate.setMilliseconds(0);

  const [currentSchedule, setCurrentSchedule] = React.useState<Schedule>(
    schedule ?? {
      startOn: defaultScheduleDate,
      days: [true, true, true, true, true, true, true],
      until: undefined,
      at: defaultScheduleDate,
    },
  );
  const [scheduleUntilValue, setScheduleUntilValue] = React.useState<string>(
    schedule?.until ? ScheduleUntilValue.ON : ScheduleUntilValue.NEVER,
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
      setCurrentSchedule((prev) => ({ ...prev, until: date }));
    } else {
      setCurrentSchedule((prev) => ({ ...prev, until: undefined }));
    }
    setScheduleUntilValue(event.target.value);
  };

  const existingBookingLabel = taskRequest
    ? getTaskBookingLabelFromTaskRequest(taskRequest)
    : undefined;
  let existingWarnTime: Date | null = null;
  if (existingBookingLabel && 'unix_millis_warn_time' in existingBookingLabel) {
    const warnTimeInt = parseInt(existingBookingLabel['unix_millis_warn_time']);
    if (!Number.isNaN(warnTimeInt)) {
      existingWarnTime = new Date(warnTimeInt);
    }
  }
  const [warnTime, setWarnTime] = React.useState<Date | null>(existingWarnTime);
  const handleWarnTimeSwitchChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    if (event.target.checked) {
      setWarnTime(new Date());
    } else {
      setWarnTime(null);
    }
  };

  const handleTaskDescriptionChange = (newCategory: string, newDesc: TaskDescription) => {
    setCurrentTaskRequest((prev) => {
      return {
        ...prev,
        category: newCategory,
        description: newDesc,
      };
    });
    setFavoriteTaskBuffer({ ...favoriteTaskBuffer, description: newDesc, category: newCategory });
  };

  // FIXME: Favorite tasks are disabled for custom compose tasks for now, as it
  // will require a re-write of FavoriteTask's pydantic model with better typing.
  const handleCustomComposeTaskDescriptionChange = (newDesc: CustomComposeTaskDescription) => {
    setCurrentTaskRequest((prev) => {
      return {
        ...prev,
        category: CustomComposeTaskDefinition.requestCategory,
        description: newDesc,
      };
    });
  };

  const onValidate = (valid: boolean) => {
    setFormFullyFilled(valid);
  };

  const renderTaskDescriptionForm = (definitionId: string) => {
    switch (definitionId) {
      case PatrolTaskDefinition.taskDefinitionId:
        return (
          <PatrolTaskForm
            taskDesc={currentTaskRequest.description as PatrolTaskDescription}
            patrolWaypoints={patrolWaypoints}
            onChange={(desc) =>
              handleTaskDescriptionChange(PatrolTaskDefinition.requestCategory, desc)
            }
            onValidate={onValidate}
          />
        );
      case DeliveryTaskDefinition.taskDefinitionId:
        return (
          <DeliveryTaskForm
            taskDesc={currentTaskRequest.description as DeliveryTaskDescription}
            pickupPoints={pickupPoints}
            dropoffPoints={dropoffPoints}
            onChange={(desc) =>
              handleTaskDescriptionChange(DeliveryTaskDefinition.requestCategory, desc)
            }
            onValidate={onValidate}
          />
        );
      case ComposeCleanTaskDefinition.taskDefinitionId:
        return (
          <ComposeCleanTaskForm
            taskDesc={currentTaskRequest.description as ComposeCleanTaskDescription}
            cleaningZones={cleaningZones}
            onChange={(desc: ComposeCleanTaskDescription) => {
              desc.category = currentTaskRequest.description.category;
              handleTaskDescriptionChange(ComposeCleanTaskDefinition.requestCategory, desc);
            }}
            onValidate={onValidate}
          />
        );
      case DeliveryPickupTaskDefinition.taskDefinitionId:
        return (
          <DeliveryPickupTaskForm
            taskDesc={currentTaskRequest.description as DeliveryPickupTaskDescription}
            pickupPoints={pickupPoints}
            cartIds={cartIds}
            dropoffPoints={dropoffPoints}
            onChange={(desc: DeliveryPickupTaskDescription) => {
              desc.category = currentTaskRequest.description.category;
              desc.phases[0].activity.description.activities[1].description.category =
                currentTaskRequest.description.category;
              handleTaskDescriptionChange(DeliveryPickupTaskDefinition.requestCategory, desc);
            }}
            onValidate={onValidate}
          />
        );
      case DeliverySequentialLotPickupTaskDefinition.taskDefinitionId:
        return (
          <DeliveryCustomTaskForm
            taskDesc={currentTaskRequest.description as DeliveryCustomTaskDescription}
            pickupZones={pickupZones}
            cartIds={cartIds}
            dropoffPoints={Object.keys(dropoffPoints)}
            onChange={(desc) => {
              desc.category = currentTaskRequest.description.category;
              desc.phases[0].activity.description.activities[1].description.category =
                currentTaskRequest.description.category;
              handleTaskDescriptionChange(
                DeliverySequentialLotPickupTaskDefinition.requestCategory,
                desc,
              );
            }}
            onValidate={onValidate}
          />
        );
      case DeliveryAreaPickupTaskDefinition.taskDefinitionId:
        return (
          <DeliveryCustomTaskForm
            taskDesc={currentTaskRequest.description as DeliveryCustomTaskDescription}
            pickupZones={pickupZones}
            cartIds={cartIds}
            dropoffPoints={Object.keys(dropoffPoints)}
            onChange={(desc) => {
              desc.category = currentTaskRequest.description.category;
              desc.phases[0].activity.description.activities[1].description.category =
                currentTaskRequest.description.category;
              handleTaskDescriptionChange(DeliveryAreaPickupTaskDefinition.requestCategory, desc);
            }}
            onValidate={onValidate}
          />
        );
      case CustomComposeTaskDefinition.taskDefinitionId:
        return (
          <CustomComposeTaskForm
            taskDesc={currentTaskRequest.description as CustomComposeTaskDescription}
            onChange={(desc) => {
              handleCustomComposeTaskDescriptionChange(desc);
            }}
            onValidate={onValidate}
          />
        );
    }
  };
  const handleTaskTypeChange = (ev: React.ChangeEvent<HTMLInputElement>) => {
    const newTaskDefinitionId = ev.target.value;
    setTaskDefinitionId(newTaskDefinitionId);

    const category = getTaskRequestCategory(newTaskDefinitionId);
    if (category === undefined) {
      const err = Error(
        `Failed to retrieve task request category for task [${newTaskDefinitionId}], there might be a misconfiguration.`,
      );
      console.error(err.message);
      onFail && onFail(err);
      return;
    }
    currentTaskRequest.category = category;

    const description = getDefaultTaskDescription(newTaskDefinitionId) ?? '';
    currentTaskRequest.description = description;

    if (
      newTaskDefinitionId !== CustomComposeTaskDefinition.taskDefinitionId &&
      typeof description === 'object'
    ) {
      setFavoriteTaskBuffer({ ...favoriteTaskBuffer, category, description });
    }
  };

  const configureTaskRequest = (scheduling: boolean): TaskRequest | null => {
    const request = { ...currentTaskRequest };
    request.requester = user;
    request.unix_millis_request_time = Date.now();

    if (taskDefinitionId === CustomComposeTaskDefinition.taskDefinitionId) {
      try {
        const obj = JSON.parse(request.description);
        request.category = CustomComposeTaskDefinition.requestCategory;
        request.description = obj;
      } catch (e) {
        console.error('Invalid custom compose task description');
        onFail && onFail(e as Error, request);
        return null;
      }
    }

    // Generate booking label for each task
    try {
      let requestBookingLabel: TaskBookingLabels | null = null;
      switch (taskDefinitionId) {
        case DeliveryPickupTaskDefinition.taskDefinitionId:
          requestBookingLabel = makeDeliveryPickupTaskBookingLabel(request.description);
          break;
        case DeliverySequentialLotPickupTaskDefinition.taskDefinitionId:
        case DeliveryAreaPickupTaskDefinition.taskDefinitionId:
          requestBookingLabel = makeDeliveryCustomTaskBookingLabel(request.description);
          break;
        case PatrolTaskDefinition.taskDefinitionId:
          requestBookingLabel = makePatrolTaskBookingLabel(request.description);
          break;
        case DeliveryTaskDefinition.taskDefinitionId:
          requestBookingLabel = makeDeliveryTaskBookingLabel(request.description);
          break;
        case ComposeCleanTaskDefinition.taskDefinitionId:
          requestBookingLabel = makeComposeCleanTaskBookingLabel(request.description);
          break;
        case CustomComposeTaskDefinition.taskDefinitionId:
          requestBookingLabel = makeCustomComposeTaskBookingLabel();
          break;
      }

      if (!requestBookingLabel) {
        const error = Error(
          `Failed to generate booking label for task request of definition ID: ${taskDefinitionId}`,
        );
        onFail && onFail(error, request);
        return null;
      }

      if (warnTime !== null) {
        requestBookingLabel['unix_millis_warn_time'] = `${warnTime.valueOf()}`;
      }

      if (scheduling) {
        requestBookingLabel['scheduled'] = 'true';
      }

      request.labels = serializeTaskBookingLabel(requestBookingLabel);
      console.log(`labels: ${request.labels}`);
    } catch (e) {
      console.error('Failed to generate string for task request label');
      onFail && onFail(e as Error, request);
      return null;
    }
    return request;
  };

  const handleSubmitNow: React.MouseEventHandler = async (ev) => {
    ev.preventDefault();
    if (!onDispatchTask) {
      return;
    }

    const request = configureTaskRequest(false);
    if (!request) {
      return;
    }

    try {
      setSubmitting(true);
      if (dispatchType === DispatchType.Robot) {
        await onDispatchTask(request, robotDispatchTarget);
      } else {
        await onDispatchTask(request, null);
      }
      setSubmitting(false);

      onSuccess && onSuccess(request);
    } catch (e) {
      setSubmitting(false);
      onFail && onFail(e as Error, request);
    }
  };

  const handleSubmitSchedule: React.FormEventHandler = async (ev) => {
    ev.preventDefault();
    if (!onScheduleTask) {
      return;
    }

    const request = configureTaskRequest(false);
    if (!request) {
      return;
    }

    try {
      setSubmitting(true);
      await onScheduleTask(request, currentSchedule);
      setSubmitting(false);
      onSuccessScheduling && onSuccessScheduling();
    } catch (e) {
      setSubmitting(false);
      onFailScheduling && onFailScheduling(e as Error);
    }
  };

  const handleEditSchedule: React.FormEventHandler = async (ev) => {
    ev.preventDefault();
    if (!onEditScheduleTask) {
      return;
    }

    const request = configureTaskRequest(false);
    if (!request) {
      return;
    }

    try {
      setSubmitting(true);
      await onEditScheduleTask(request, currentSchedule);
      setSubmitting(false);
      onSuccessScheduling && onSuccessScheduling();
    } catch (e) {
      setSubmitting(false);
      onFailScheduling && onFailScheduling(e as Error);
    }
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
      favoriteTask.task_definition_id = taskDefinitionId ?? tasksToDisplay[0].taskDefinitionId;

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

      const defaultTaskRequest = getDefaultTaskRequest(tasksToDisplay[0].taskDefinitionId);
      if (!defaultTaskRequest) {
        // We should never reach this area as we have already validated that
        // each supported task have a valid task request for generation
        console.error('Failed to reset task request buffer after deleting favorite task');
        return;
      }

      setCurrentTaskRequest(defaultTaskRequest);
      setOpenFavoriteDialog(false);
      setCallToDeleteFavoriteTask(false);
      setCallToUpdateFavoriteTask(false);
    } catch (e) {
      setDeletingFavoriteTask(false);
      onFailFavoriteTask && onFailFavoriteTask(e as Error, favoriteTaskBuffer);
    }
  };

  const handlePrioritySwitchChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setCurrentTaskRequest((prev) => {
      return {
        ...prev,
        priority: createTaskPriority(event.target.checked),
      };
    });
  };

  const enum DispatchType {
    Automatic = 'Automatic',
    Fleet = 'Fleet',
    Robot = 'Robot',
  }

  const [dispatchType, setDispatchType] = React.useState<DispatchType>(
    taskRequest && taskRequest.fleet_name ? DispatchType.Fleet : DispatchType.Automatic,
  );
  const [robotDispatchTarget, setRobotDispatchTarget] = React.useState<RobotDispatchTarget | null>(
    null,
  );

  const handleChangeDispatchType = (ev: React.ChangeEvent<HTMLInputElement>) => {
    setDispatchType(ev.target.value as DispatchType);
    setCurrentTaskRequest((prev) => {
      return {
        ...prev,
        fleet_name: undefined,
      };
    });
    setRobotDispatchTarget(null);
  };

  const handleDispatchFleetTargetChange = (ev: React.ChangeEvent<HTMLInputElement>) => {
    setCurrentTaskRequest((prev) => {
      return {
        ...prev,
        fleet_name: ev.target.value.length > 0 ? ev.target.value : undefined,
      };
    });
  };

  const handleDispatchRobotTargetChange = (ev: React.ChangeEvent<HTMLInputElement>) => {
    if (!fleets) {
      setRobotDispatchTarget(null);
      return;
    }
    let robotFleet: string | null = null;
    for (const fleetName of Object.keys(fleets)) {
      if (fleets[fleetName].includes(ev.target.value)) {
        robotFleet = fleetName;
        break;
      }
    }
    if (robotFleet === null) {
      // Technically this will never happen, as users can only select robots
      // that have fleets already registered.
      console.error(`Failed to find fleet name for robot [${ev.target.value}]`);
      return;
    }
    setRobotDispatchTarget({ fleet: robotFleet, robot: ev.target.value });
  };

  return (
    <>
      <StyledDialog
        title={taskRequest ? 'Edit Schedule' : 'Create Task'}
        maxWidth={isScreenHeightLessThan800 ? 'md' : 'lg'}
        disableEnforceFocus
        {...otherProps}
      >
        <form aria-label="task-form">
          <DialogTitle>
            <Grid container wrap="nowrap" alignItems="center">
              <Grid item xs className={classes.title} container justifyContent="center">
                {taskRequest ? 'Edit Schedule' : 'Create Task'}
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
                        setCurrentTaskRequest({
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
                <Grid container spacing={theme.spacing(2)} alignItems="center">
                  <Grid item xs={12}>
                    <TextField
                      select
                      id="task-type"
                      label="Task Category"
                      variant="outlined"
                      fullWidth
                      margin="normal"
                      value={taskDefinitionId}
                      onChange={handleTaskTypeChange}
                      sx={{
                        '& .MuiInputBase-input': {
                          fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
                          height: isScreenHeightLessThan800 ? '1.5rem' : '3.5rem',
                        },
                      }}
                      InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
                    >
                      {validTasks.map((taskDefinition) => {
                        return (
                          <MenuItem
                            value={taskDefinition.taskDefinitionId}
                            key={taskDefinition.taskDefinitionId}
                          >
                            {taskDefinition.taskDisplayName}
                          </MenuItem>
                        );
                      })}
                    </TextField>
                  </Grid>
                  <Grid item xs={1}>
                    <Tooltip title="Ongoing tasks where estimated finish times are past their specified warning times will be flagged.">
                      <Switch checked={warnTime !== null} onChange={handleWarnTimeSwitchChange} />
                    </Tooltip>
                  </Grid>
                  <Grid item xs={8}>
                    <DateTimePicker
                      disabled={warnTime === null}
                      value={warnTime}
                      onChange={(date) => {
                        setWarnTime(date);
                      }}
                      label="Warn Time"
                      sx={{ marginLeft: theme.spacing(1) }}
                    />
                  </Grid>
                  <Grid item xs={3} justifyContent="flex-end">
                    <Tooltip title="Prioritized tasks will added to the front of the task execution queue">
                      <FormControlLabel
                        control={
                          <Switch
                            checked={parseTaskPriority(currentTaskRequest.priority)}
                            onChange={handlePrioritySwitchChange}
                          />
                        }
                        label="Prioritize"
                        sx={{
                          color: parseTaskPriority(currentTaskRequest.priority)
                            ? undefined
                            : theme.palette.action.disabled,
                        }}
                      />
                    </Tooltip>
                  </Grid>
                </Grid>
                <Divider
                  orientation="horizontal"
                  flexItem
                  style={{ marginTop: theme.spacing(2), marginBottom: theme.spacing(2) }}
                />
                <Grid container spacing={theme.spacing(2)} alignItems="center">
                  <Grid item xs={dispatchType === DispatchType.Automatic ? 12 : 6}>
                    <TextField
                      select
                      id="dispatch-type"
                      label="Dispatch Type"
                      variant="outlined"
                      fullWidth
                      margin="normal"
                      value={dispatchType}
                      onChange={handleChangeDispatchType}
                    >
                      <MenuItem value={DispatchType.Automatic}>{DispatchType.Automatic}</MenuItem>
                      <MenuItem value={DispatchType.Fleet}>{DispatchType.Fleet}</MenuItem>
                      <MenuItem value={DispatchType.Robot}>{DispatchType.Robot}</MenuItem>
                    </TextField>
                  </Grid>
                  <Grid item xs={6}>
                    {dispatchType === DispatchType.Fleet ? (
                      <TextField
                        select
                        id="dispatch-fleet-type"
                        label="Select Fleet"
                        variant="outlined"
                        fullWidth
                        margin="normal"
                        value={currentTaskRequest.fleet_name ?? ''}
                        defaultValue=""
                        onChange={handleDispatchFleetTargetChange}
                      >
                        {fleets &&
                          Object.keys(fleets).map((fleetName) => {
                            return (
                              <MenuItem value={fleetName} key={fleetName}>
                                {fleetName}
                              </MenuItem>
                            );
                          })}
                      </TextField>
                    ) : dispatchType === DispatchType.Robot ? (
                      <TextField
                        select
                        id="dispatch-robot-type"
                        label="Select Robot"
                        variant="outlined"
                        fullWidth
                        margin="normal"
                        value={robotDispatchTarget?.robot ?? ''}
                        defaultValue=""
                        onChange={handleDispatchRobotTargetChange}
                      >
                        {fleets &&
                          Object.keys(fleets).flatMap((fleetName) => {
                            const fleetRobots = [
                              <Divider key={`${fleetName}_divider`} />,
                              <MenuItem
                                value={fleetName}
                                key={fleetName}
                                dense
                                sx={{ textDecoration: 'underline' }}
                                disabled
                              >
                                {fleetName}
                              </MenuItem>,
                            ];
                            return fleetRobots.concat(
                              fleets[fleetName].map((robotName) => (
                                <MenuItem
                                  value={robotName}
                                  key={robotName}
                                  sx={{ pl: theme.spacing(4) }}
                                >
                                  {robotName}
                                </MenuItem>
                              )),
                            );
                          })}
                      </TextField>
                    ) : null}
                  </Grid>
                </Grid>
                <Divider
                  orientation="horizontal"
                  flexItem
                  sx={{ marginTop: theme.spacing(2), marginBottom: theme.spacing(2) }}
                />
                {renderTaskDescriptionForm(taskDefinitionId)}
              </Grid>
            </Grid>
          </DialogContent>
          <DialogActions>
            <Button
              aria-label="Save as a favorite task"
              variant={callToUpdateFavoriteTask ? 'contained' : 'outlined'}
              color="primary"
              size={isScreenHeightLessThan800 ? 'small' : 'medium'}
              startIcon={callToUpdateFavoriteTask ? <SaveIcon /> : <FavoriteBorder />}
              onClick={() => {
                !callToUpdateFavoriteTask &&
                  setFavoriteTaskBuffer({ ...favoriteTaskBuffer, name: '', id: '' });
                setOpenFavoriteDialog(true);
              }}
              // FIXME: Favorite tasks are disabled for custom compose
              // tasks for now, as it will require a re-write of
              // FavoriteTask's pydantic model with better typing.
              disabled={taskDefinitionId === CustomComposeTaskDefinition.taskDefinitionId}
            >
              {callToUpdateFavoriteTask ? `Confirm edits` : 'Save as a favorite task'}
            </Button>
          </DialogActions>
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
            {onScheduleTask ? (
              <Button
                variant="contained"
                color="primary"
                disabled={submitting || !formFullyFilled || robotDispatchTarget !== null}
                className={classes.actionBtn}
                onClick={() => setOpenSchedulingDialog(true)}
                size={isScreenHeightLessThan800 ? 'small' : 'medium'}
                startIcon={<ScheduleSendIcon />}
              >
                Add to Schedule
              </Button>
            ) : null}
            {onEditScheduleTask ? (
              <Button
                variant="contained"
                color="primary"
                disabled={submitting || !formFullyFilled || robotDispatchTarget !== null}
                className={classes.actionBtn}
                onClick={() => setOpenSchedulingDialog(true)}
                size={isScreenHeightLessThan800 ? 'small' : 'medium'}
                startIcon={<ScheduleSendIcon />}
              >
                Edit schedule
              </Button>
            ) : null}
            <Button
              variant="contained"
              type="submit"
              color="primary"
              disabled={submitting || !formFullyFilled || schedule !== undefined}
              className={classes.actionBtn}
              aria-label="Submit Now"
              onClick={handleSubmitNow}
              size={isScreenHeightLessThan800 ? 'small' : 'medium'}
              startIcon={<SendIcon />}
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
              fullWidth
            />
          )}
          {callToDeleteFavoriteTask && (
            <Typography>{`Are you sure you want to delete "${favoriteTaskBuffer.name}"?`}</Typography>
          )}
        </ConfirmationDialog>
      )}
      {openSchedulingDialog && (
        <ConfirmationDialog
          confirmText={schedule ? 'Edit Schedule' : 'Schedule'}
          cancelText="Cancel"
          open={openSchedulingDialog}
          title={schedule ? 'Edit Scheduled Task' : 'Schedule Task'}
          submitting={false}
          onClose={() => setOpenSchedulingDialog(false)}
          onSubmit={(ev) => {
            if (schedule) {
              handleEditSchedule(ev);
            } else {
              handleSubmitSchedule(ev);
            }
            setOpenSchedulingDialog(false);
          }}
        >
          <Grid container spacing={theme.spacing(2)} marginTop={theme.spacing(1)}>
            <Grid item xs={6}>
              <DatePicker
                value={currentSchedule.startOn}
                onChange={(date) => {
                  if (!date) {
                    console.error('DatePicker: invalid date');
                    return;
                  }
                  console.debug(`DatePicker: ${date}`);
                  setCurrentSchedule((prev) => {
                    date.setHours(currentSchedule.at.getHours());
                    date.setMinutes(currentSchedule.at.getMinutes());
                    date.setSeconds(0);
                    date.setMilliseconds(0);
                    console.debug(`DatePicker setCurrentSchedule: ${date}`);
                    return { ...prev, startOn: date };
                  });
                }}
                label="Start On"
              />
            </Grid>
            <Grid item xs={6}>
              <TimePicker
                value={currentSchedule.at}
                onChange={(date) => {
                  if (!date) {
                    console.error('TimePicker: invalid date');
                    return;
                  }
                  console.debug(`TimePicker: ${date}`);
                  if (!isNaN(date.valueOf())) {
                    setCurrentSchedule((prev) => {
                      const startOn = new Date(prev.startOn);
                      startOn.setHours(date.getHours());
                      startOn.setMinutes(date.getMinutes());
                      startOn.setSeconds(0);
                      startOn.setMilliseconds(0);
                      console.debug(`TimePicker setCurrentSchedule date: ${date}`);
                      console.debug(`TimePicker setCurrentSchedule startOn: ${startOn}`);
                      return { ...prev, at: date, startOn };
                    });
                  }
                }}
                label="At"
              />
            </Grid>
            <Grid item xs={12}>
              <DaySelectorSwitch
                value={currentSchedule.days}
                onChange={(days) => setCurrentSchedule((prev) => ({ ...prev, days }))}
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
                      scheduleUntilValue === ScheduleUntilValue.NEVER
                        ? new Date()
                        : currentSchedule.until
                    }
                    onChange={(date) => {
                      if (!date) {
                        console.error('Until DatePicker: invalid date');
                        return;
                      }
                      console.debug(`Until DatePicker: ${date}`);
                      setCurrentSchedule((prev) => {
                        date.setHours(23);
                        date.setMinutes(59);
                        console.debug(`Until DatePicker setCurrentSchedule: ${date}`);
                        return { ...prev, until: date };
                      });
                    }}
                    disabled={scheduleUntilValue !== ScheduleUntilValue.ON}
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
