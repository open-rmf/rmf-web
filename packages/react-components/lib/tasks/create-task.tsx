/**
 * FIXME(kp): Make the whole task request system task agnostic.
 * For that RMF needs to support task discovery and UI schemas https://github.com/open-rmf/rmf_api_msgs/issues/32.
 */

import UpdateIcon from '@mui/icons-material/Create';
import DeleteIcon from '@mui/icons-material/Delete';
import PlaceOutlined from '@mui/icons-material/PlaceOutlined';
import {
  Autocomplete,
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
  ListItemIcon,
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
import type { TaskFavoritePydantic as TaskFavorite, TaskRequest } from 'api-client';
import React from 'react';
import { Loading } from '..';
import { ConfirmationDialog, ConfirmationDialogProps } from '../confirmation-dialog';
import { PositiveIntField } from '../form-inputs';

// A bunch of manually defined descriptions to avoid using `any`.
interface PatrolTaskDescription {
  places: string[];
  rounds: number;
}

interface LotPickupActivity {
  category: string;
  description: {
    unix_millis_action_duration_estimate: number;
    category: string;
    description: {
      cart_id: string;
      pickup_lot: string;
    };
  };
}

interface ZonePickupActivity {
  category: string;
  description: {
    unix_millis_action_duration_estimate: number;
    category: string;
    description: {
      cart_id: string;
      pickup_zone: string;
    };
  };
}

interface GoToPlaceActivity {
  category: string;
  description: string;
}

interface DropoffActivity {
  category: string;
  description: {
    unix_millis_action_duration_estimate: number;
    category: string;
    description: {};
  };
}

interface DeliveryCustomPhase {
  activity: {
    category: string;
    description: {
      activities: [
        go_to_pickup: GoToPlaceActivity,
        pickup_cart: ZonePickupActivity,
        go_to_place: GoToPlaceActivity,
        dropoff_cart: DropoffActivity,
      ];
    };
  };
}

interface DeliveryCustomTaskDescription {
  category: string;
  phases: [deliverySequence: DeliveryCustomPhase];
}

interface DeliveryPhase {
  activity: {
    category: string;
    description: {
      activities: [
        go_to_pickup: GoToPlaceActivity,
        pickup_cart: LotPickupActivity,
        go_to_dropoff: GoToPlaceActivity,
        dropoff_cart: DropoffActivity,
      ];
    };
  };
}

interface DeliveryTaskDescription {
  category: string;
  phases: [deliverySequence: DeliveryPhase];
}

type TaskDescription =
  | DeliveryTaskDescription
  | DeliveryCustomTaskDescription
  | PatrolTaskDescription;

const isNonEmptyString = (value: string): boolean => value.length > 0;

const isDeliveryTaskDescriptionValid = (
  taskDescription: DeliveryTaskDescription,
  pickupPoints: Record<string, string>,
  dropoffPoints: Record<string, string>,
): boolean => {
  const goToPickup = taskDescription.phases[0].activity.description.activities[0];
  const pickup = taskDescription.phases[0].activity.description.activities[1];
  const goToDropoff = taskDescription.phases[0].activity.description.activities[2];
  return (
    isNonEmptyString(goToPickup.description) &&
    Object.keys(pickupPoints).includes(goToPickup.description) &&
    pickupPoints[goToPickup.description] === pickup.description.description.pickup_lot &&
    isNonEmptyString(pickup.description.description.cart_id) &&
    isNonEmptyString(goToDropoff.description) &&
    Object.keys(dropoffPoints).includes(goToDropoff.description)
  );
};

const isDeliveryCustomTaskDescriptionValid = (
  taskDescription: DeliveryCustomTaskDescription,
  pickupZones: string[],
  dropoffPoints: string[],
): boolean => {
  const goToPickup = taskDescription.phases[0].activity.description.activities[0];
  const pickup = taskDescription.phases[0].activity.description.activities[1];
  const goToDropoff = taskDescription.phases[0].activity.description.activities[2];
  return (
    isNonEmptyString(goToPickup.description) &&
    isNonEmptyString(pickup.description.description.pickup_zone) &&
    pickupZones.includes(pickup.description.description.pickup_zone) &&
    isNonEmptyString(pickup.description.description.cart_id) &&
    isNonEmptyString(goToDropoff.description) &&
    dropoffPoints.includes(goToDropoff.description)
  );
};

const isPatrolTaskDescriptionValid = (taskDescription: PatrolTaskDescription): boolean => {
  if (taskDescription.places.length === 0) {
    return false;
  }
  for (const place of taskDescription.places) {
    if (place.length === 0) {
      return false;
    }
  }
  return taskDescription.rounds > 0;
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
  if (taskRequest.category === 'patrol') {
    const formattedPlaces = taskRequest.description.places.map((place: string) => `[${place}]`);
    return `[Patrol] [${taskRequest.description.rounds}] round/s, along ${formattedPlaces.join(
      ', ',
    )}`;
  }

  const goToPickup: GoToPlaceActivity =
    taskRequest.description.phases[0].activity.description.activities[0];
  const pickup: LotPickupActivity =
    taskRequest.description.phases[0].activity.description.activities[1];
  const cartId = pickup.description.description.cart_id;
  const goToDropoff: GoToPlaceActivity =
    taskRequest.description.phases[0].activity.description.activities[2];

  switch (taskRequest.description.category) {
    case 'delivery_pickup': {
      return `[Delivery - 1:1] payload [${cartId}] from [${goToPickup.description}] to [${goToDropoff.description}]`;
    }
    case 'delivery_sequential_lot_pickup': {
      return `[Delivery - Sequential lot pick up] payload [${cartId}] from [${goToPickup.description}] to [${goToDropoff.description}]`;
    }
    case 'delivery_area_pickup': {
      return `[Delivery - Area pick up] payload [${cartId}] from [${goToPickup.description}] to [${goToDropoff.description}]`;
    }
    default:
      return `[Unknown] type "${taskRequest.description.category}"`;
  }
}

interface DeliveryTaskFormProps {
  taskDesc: DeliveryTaskDescription;
  pickupPoints: Record<string, string>;
  cartIds: string[];
  dropoffPoints: Record<string, string>;
  onChange(taskDesc: TaskDescription): void;
  allowSubmit(allow: boolean): void;
}

function DeliveryTaskForm({
  taskDesc,
  pickupPoints = {},
  cartIds = [],
  dropoffPoints = {},
  onChange,
  allowSubmit,
}: DeliveryTaskFormProps) {
  const theme = useTheme();
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
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
          options={Object.keys(pickupPoints).sort()}
          value={taskDesc.phases[0].activity.description.activities[0].description}
          onInputChange={(_ev, newValue) => {
            const pickupLot = pickupPoints[newValue] ?? '';
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0].activity.description.activities[0].description = newValue;
            newTaskDesc.phases[0].activity.description.activities[1].description.description.pickup_lot =
              pickupLot;
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const place = (ev.target as HTMLInputElement).value;
            const pickupLot = pickupPoints[place] ?? '';
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0].activity.description.activities[0].description = place;
            newTaskDesc.phases[0].activity.description.activities[1].description.description.pickup_lot =
              pickupLot;
            onInputChange(newTaskDesc);
          }}
          sx={{
            '& .MuiOutlinedInput-root': {
              height: isScreenHeightLessThan800 ? '3rem' : '3.5rem',
              fontSize: isScreenHeightLessThan800 ? 14 : 20,
            },
          }}
          renderInput={(params) => (
            <TextField
              {...params}
              label="Pickup Location"
              required
              InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 14 : 20 } }}
              error={
                !Object.keys(pickupPoints).includes(
                  taskDesc.phases[0].activity.description.activities[0].description,
                )
              }
            />
          )}
        />
      </Grid>
      <Grid item xs={4}>
        <Autocomplete
          id="cart_id"
          freeSolo
          fullWidth
          options={cartIds}
          value={
            taskDesc.phases[0].activity.description.activities[1].description.description.cart_id
          }
          getOptionLabel={(option) => option}
          onInputChange={(_ev, newValue) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0].activity.description.activities[1].description.description.cart_id =
              newValue;
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0].activity.description.activities[1].description.description.cart_id =
              (ev.target as HTMLInputElement).value;
            onInputChange(newTaskDesc);
          }}
          sx={{
            '& .MuiOutlinedInput-root': {
              height: isScreenHeightLessThan800 ? '3rem' : '3.5rem',
              fontSize: isScreenHeightLessThan800 ? 14 : 20,
            },
          }}
          renderInput={(params) => (
            <TextField
              {...params}
              label="Cart ID"
              required
              InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 14 : 20 } }}
              error={
                taskDesc.phases[0].activity.description.activities[1].description.description
                  .cart_id.length === 0
              }
            />
          )}
        />
      </Grid>
      <Grid item xs={8}>
        <Autocomplete
          id="dropoff-location"
          freeSolo
          fullWidth
          options={Object.keys(dropoffPoints).sort()}
          value={taskDesc.phases[0].activity.description.activities[2].description}
          onInputChange={(_ev, newValue) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0].activity.description.activities[2].description = newValue;
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0].activity.description.activities[2].description = (
              ev.target as HTMLInputElement
            ).value;
            onInputChange(newTaskDesc);
          }}
          sx={{
            '& .MuiOutlinedInput-root': {
              height: isScreenHeightLessThan800 ? '3rem' : '3.5rem',
              fontSize: isScreenHeightLessThan800 ? 14 : 20,
            },
          }}
          renderInput={(params) => (
            <TextField
              {...params}
              label="Dropoff Location"
              required
              InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 14 : 20 } }}
              error={
                !Object.keys(dropoffPoints).includes(
                  taskDesc.phases[0].activity.description.activities[2].description,
                )
              }
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
  cartIds: string[];
  dropoffPoints: string[];
  onChange(taskDesc: DeliveryCustomTaskDescription): void;
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
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
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
          options={pickupZones.sort()}
          value={taskDesc.phases[0].activity.description.activities[0].description}
          onInputChange={(_ev, newValue) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0].activity.description.activities[0].description = newValue;
            newTaskDesc.phases[0].activity.description.activities[1].description.description.pickup_zone =
              newValue;
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const zone = (ev.target as HTMLInputElement).value;
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0].activity.description.activities[0].description = zone;
            newTaskDesc.phases[0].activity.description.activities[1].description.description.pickup_zone =
              zone;
            onInputChange(newTaskDesc);
          }}
          sx={{
            '& .MuiOutlinedInput-root': {
              height: isScreenHeightLessThan800 ? '3rem' : '3.5rem',
              fontSize: isScreenHeightLessThan800 ? 14 : 20,
            },
          }}
          renderInput={(params) => (
            <TextField
              {...params}
              label="Pickup Zone"
              required
              InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 14 : 20 } }}
              error={
                !pickupZones.includes(
                  taskDesc.phases[0].activity.description.activities[0].description,
                )
              }
            />
          )}
        />
      </Grid>
      <Grid item xs={4}>
        <Autocomplete
          id="cart-id"
          freeSolo
          fullWidth
          options={cartIds}
          value={
            taskDesc.phases[0].activity.description.activities[1].description.description.cart_id
          }
          getOptionLabel={(option) => option}
          onInputChange={(_ev, newValue) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0].activity.description.activities[1].description.description.cart_id =
              newValue;
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0].activity.description.activities[1].description.description.cart_id =
              (ev.target as HTMLInputElement).value;
            onInputChange(newTaskDesc);
          }}
          sx={{
            '& .MuiOutlinedInput-root': {
              height: isScreenHeightLessThan800 ? '3rem' : '3.5rem',
              fontSize: isScreenHeightLessThan800 ? 14 : 20,
            },
          }}
          renderInput={(params) => (
            <TextField
              {...params}
              label="Cart ID"
              required
              InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 14 : 20 } }}
              error={
                taskDesc.phases[0].activity.description.activities[1].description.description
                  .cart_id.length === 0
              }
            />
          )}
        />
      </Grid>
      <Grid item xs={8}>
        <Autocomplete
          id="dropoff-location"
          freeSolo
          fullWidth
          options={dropoffPoints.sort()}
          value={taskDesc.phases[0].activity.description.activities[2].description}
          onInputChange={(_ev, newValue) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0].activity.description.activities[2].description = newValue;
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0].activity.description.activities[2].description = (
              ev.target as HTMLInputElement
            ).value;
            onInputChange(newTaskDesc);
          }}
          sx={{
            '& .MuiOutlinedInput-root': {
              height: isScreenHeightLessThan800 ? '3rem' : '3.5rem',
              fontSize: isScreenHeightLessThan800 ? 14 : 20,
            },
          }}
          renderInput={(params) => (
            <TextField
              {...params}
              label="Dropoff Location"
              required
              InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 14 : 20 } }}
              error={
                !dropoffPoints.includes(
                  taskDesc.phases[0].activity.description.activities[2].description,
                )
              }
            />
          )}
        />
      </Grid>
    </Grid>
  );
}

interface PlaceListProps {
  places: string[];
  onClick(places_index: number): void;
}

function PlaceList({ places, onClick }: PlaceListProps) {
  const theme = useTheme();
  return (
    <List
      dense
      sx={{
        bgcolor: 'background.paper',
        marginLeft: theme.spacing(3),
        marginRight: theme.spacing(3),
      }}
    >
      {places.map((value, index) => (
        <ListItem
          key={`${value}-${index}`}
          secondaryAction={
            <IconButton edge="end" aria-label="delete" onClick={() => onClick(index)}>
              <DeleteIcon />
            </IconButton>
          }
        >
          <ListItemIcon>
            <PlaceOutlined />
          </ListItemIcon>
          <ListItemText primary={`Place Name:   ${value}`} />
        </ListItem>
      ))}
    </List>
  );
}

interface PatrolTaskFormProps {
  taskDesc: PatrolTaskDescription;
  patrolWaypoints: string[];
  onChange(patrolTaskDescription: PatrolTaskDescription): void;
  allowSubmit(allow: boolean): void;
}

function PatrolTaskForm({ taskDesc, patrolWaypoints, onChange, allowSubmit }: PatrolTaskFormProps) {
  const theme = useTheme();
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  const onInputChange = (desc: PatrolTaskDescription) => {
    allowSubmit(isPatrolTaskDescriptionValid(desc));
    onChange(desc);
  };

  return (
    <Grid container spacing={theme.spacing(2)} justifyContent="center" alignItems="center">
      <Grid item xs={isScreenHeightLessThan800 ? 8 : 10}>
        <Autocomplete
          id="place-input"
          freeSolo
          fullWidth
          options={patrolWaypoints.sort()}
          onChange={(_ev, newValue) =>
            newValue !== null &&
            onInputChange({
              ...taskDesc,
              places: taskDesc.places.concat(newValue).filter((el: string) => el),
            })
          }
          sx={{
            '& .MuiOutlinedInput-root': {
              height: isScreenHeightLessThan800 ? '3rem' : '3.5rem',
              fontSize: isScreenHeightLessThan800 ? 14 : 20,
            },
          }}
          renderInput={(params) => (
            <TextField
              {...params}
              label="Place Name"
              required={true}
              InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 14 : 20 } }}
            />
          )}
        />
      </Grid>
      <Grid item xs={isScreenHeightLessThan800 ? 4 : 2}>
        <PositiveIntField
          id="loops"
          label="Loops"
          sx={{
            '& .MuiOutlinedInput-root': {
              height: isScreenHeightLessThan800 ? '3rem' : '3.5rem',
              fontSize: isScreenHeightLessThan800 ? 14 : 20,
            },
          }}
          value={taskDesc.rounds}
          onChange={(_ev, val) => {
            onInputChange({
              ...taskDesc,
              rounds: val,
            });
          }}
        />
      </Grid>
      <Grid item xs={10}>
        <PlaceList
          places={taskDesc && taskDesc.places ? taskDesc.places : []}
          onClick={(places_index) =>
            taskDesc.places.splice(places_index, 1) &&
            onInputChange({
              ...taskDesc,
            })
          }
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

function defaultDeliveryTaskDescription(): DeliveryTaskDescription {
  return {
    category: 'delivery_pickup',
    phases: [
      {
        activity: {
          category: 'sequence',
          description: {
            activities: [
              {
                category: 'go_to_place',
                description: '',
              },
              {
                category: 'perform_action',
                description: {
                  unix_millis_action_duration_estimate: 60000,
                  category: 'delivery_pickup',
                  description: {
                    cart_id: '',
                    pickup_lot: '',
                  },
                },
              },
              {
                category: 'go_to_place',
                description: '',
              },
              {
                category: 'perform_action',
                description: {
                  unix_millis_action_duration_estimate: 60000,
                  category: 'delivery_dropoff',
                  description: {},
                },
              },
            ],
          },
        },
      },
    ],
  };
}

function defaultDeliveryCustomTaskDescription(taskCategory: string): DeliveryCustomTaskDescription {
  return {
    category: taskCategory,
    phases: [
      {
        activity: {
          category: 'sequence',
          description: {
            activities: [
              {
                category: 'go_to_place',
                description: '',
              },
              {
                category: 'perform_action',
                description: {
                  unix_millis_action_duration_estimate: 60000,
                  category: taskCategory,
                  description: {
                    cart_id: '',
                    pickup_zone: '',
                  },
                },
              },
              {
                category: 'go_to_place',
                description: '',
              },
              {
                category: 'perform_action',
                description: {
                  unix_millis_action_duration_estimate: 60000,
                  category: 'delivery_dropoff',
                  description: {},
                },
              },
            ],
          },
        },
      },
    ],
  };
}

function defaultPatrolTask(): PatrolTaskDescription {
  return {
    places: [],
    rounds: 1,
  };
}

function defaultTaskDescription(taskCategory: string): TaskDescription | undefined {
  switch (taskCategory) {
    case 'delivery_pickup':
      return defaultDeliveryTaskDescription();
    case 'delivery_sequential_lot_pickup':
    case 'delivery_area_pickup':
      return defaultDeliveryCustomTaskDescription(taskCategory);
    case 'patrol':
      return defaultPatrolTask();
    default:
      return undefined;
  }
}

function defaultTask(): TaskRequest {
  return {
    category: 'compose',
    description: defaultDeliveryTaskDescription(),
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
    description: defaultDeliveryTaskDescription(),
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
  cartIds?: string[];
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

  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

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

  const handleTaskDescriptionChange = (newDesc: TaskDescription) => {
    taskRequest.category = 'compose';
    taskRequest.description = newDesc;
    setFavoriteTaskBuffer({ ...favoriteTaskBuffer, description: newDesc, category: 'compose' });
    updateTasks();
  };

  const legacyHandleTaskDescriptionChange = (newCategory: string, newDesc: TaskDescription) => {
    taskRequest.category = newCategory;
    taskRequest.description = newDesc;
    setFavoriteTaskBuffer({ ...favoriteTaskBuffer, description: newDesc, category: newCategory });
    updateTasks();
  };

  const renderTaskDescriptionForm = () => {
    if (taskRequest.category === 'patrol') {
      return (
        <PatrolTaskForm
          taskDesc={taskRequest.description as PatrolTaskDescription}
          patrolWaypoints={patrolWaypoints}
          onChange={(desc) => legacyHandleTaskDescriptionChange('patrol', desc)}
          allowSubmit={allowSubmit}
        />
      );
    }

    switch (taskRequest.description.category) {
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
              handleTaskDescriptionChange(desc);
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
              handleTaskDescriptionChange(desc);
            }}
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
    const category = newCategory === 'patrol' ? 'patrol' : 'compose';
    taskRequest.category = category;

    setFavoriteTaskBuffer({ ...favoriteTaskBuffer, category, description: newDesc });

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

      // Workaround where all the task category need to be compose.
      if (t.category !== 'patrol') {
        t.category = 'compose';
      }
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
        maxWidth={isScreenHeightLessThan800 ? 'md' : 'lg'}
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
                      <MenuItem
                        value="delivery_pickup"
                        disabled={
                          Object.keys(pickupPoints).length === 0 ||
                          Object.keys(dropoffPoints).length === 0
                        }
                        sx={{
                          '& .MuiMenu-list': {
                            fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
                          },
                        }}
                      >
                        Delivery - 1:1
                      </MenuItem>
                      <MenuItem
                        value="delivery_sequential_lot_pickup"
                        disabled={Object.keys(dropoffPoints).length === 0}
                      >
                        Delivery - Sequential lot pick up
                      </MenuItem>
                      <MenuItem
                        value="delivery_area_pickup"
                        disabled={Object.keys(dropoffPoints).length === 0}
                      >
                        Delivery - Area pick up
                      </MenuItem>
                      <MenuItem
                        value="patrol"
                        disabled={!patrolWaypoints || patrolWaypoints.length === 0}
                      >
                        Patrol
                      </MenuItem>
                    </TextField>
                  </Grid>
                  <Grid item xs={isScreenHeightLessThan800 ? 6 : 7}>
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
                        updateTasks();
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
              aria-label={submitText}
              onClick={handleSubmitNow}
              size={isScreenHeightLessThan800 ? 'small' : 'medium'}
            >
              <Loading
                hideChildren
                loading={submitting}
                size={isScreenHeightLessThan800 ? '0.8em' : '1.5em'}
                color="inherit"
              >
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
                onChange={(date) => {
                  if (!date) {
                    console.error('DatePicker: invalid date');
                    return;
                  }
                  console.debug(`DatePicker: ${date}`);
                  setSchedule((prev) => {
                    date.setHours(schedule.at.getHours());
                    date.setMinutes(schedule.at.getMinutes());
                    console.debug(`DatePicker setSchedule: ${date}`);
                    return { ...prev, startOn: date };
                  });
                }}
                label="Start On"
                disabled={!scheduleEnabled}
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
                      console.debug(`TimePicker setSchedule: ${date}`);
                      return { ...prev, at: date, startOn };
                    });
                  }
                }}
                label="At"
                disabled={!scheduleEnabled}
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
