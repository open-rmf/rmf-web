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
import type { TaskBookingLabel, TaskFavorite, TaskRequest } from 'api-client';
import React from 'react';
import { Loading } from '..';
import { ConfirmationDialog, ConfirmationDialogProps } from '../confirmation-dialog';
import { PositiveIntField } from '../form-inputs';
import { serializeTaskBookingLabel } from './task-booking-label-utils';

interface TaskDefinition {
  task_definition_id: string;
  task_display_name: string;
}

const SupportedTaskDefinitionsMap: Record<string, TaskDefinition> = {
  delivery_pickup: {
    task_definition_id: 'delivery_pickup',
    task_display_name: 'Delivery - 1:1',
  },
  delivery_sequential_lot_pickup: {
    task_definition_id: 'delivery_sequential_lot_pickup',
    task_display_name: 'Delivery - Sequential lot pick up',
  },
  delivery_area_pickup: {
    task_definition_id: 'delivery_area_pickup',
    task_display_name: 'Delivery - Area pick up',
  },
  patrol: {
    task_definition_id: 'patrol',
    task_display_name: 'Patrol',
  },
  custom_compose: {
    task_definition_id: 'custom_compose',
    task_display_name: 'Custom Compose Task',
  },
};

// If no task definition id is found in a past task (scheduled or favorite)
const DefaultTaskDefinitionId = 'custom_compose';

// FIXME: This is the order of the task type dropdown, and will be migrated out
// as a build-time configuration in a subsequent patch.
const SupportedTaskDefinitions: TaskDefinition[] = [
  {
    task_definition_id: 'delivery_pickup',
    task_display_name: 'Delivery - 1:1',
  },
  {
    task_definition_id: 'delivery_sequential_lot_pickup',
    task_display_name: 'Delivery - Sequential lot pick up',
  },
  {
    task_definition_id: 'delivery_area_pickup',
    task_display_name: 'Delivery - Area pick up',
  },
  {
    task_definition_id: 'patrol',
    task_display_name: 'Patrol',
  },
  {
    task_definition_id: 'custom_compose',
    task_display_name: 'Custom Compose Task',
  },
];

function makeDeliveryTaskBookingLabel(task_description: DeliveryTaskDescription): TaskBookingLabel {
  const pickupDescription =
    task_description.phases[0].activity.description.activities[1].description.description;
  return {
    description: {
      task_definition_id: task_description.category,
      pickup: pickupDescription.pickup_lot,
      destination: task_description.phases[1].activity.description.activities[0].description,
      cart_id: pickupDescription.cart_id,
    },
  };
}

function makeDeliveryCustomTaskBookingLabel(
  task_description: DeliveryCustomTaskDescription,
): TaskBookingLabel {
  const pickupDescription =
    task_description.phases[0].activity.description.activities[1].description.description;
  return {
    description: {
      task_definition_id: task_description.category,
      pickup: pickupDescription.pickup_zone,
      destination: task_description.phases[1].activity.description.activities[0].description,
      cart_id: pickupDescription.cart_id,
    },
  };
}

function makePatrolTaskBookingLabel(task_description: PatrolTaskDescription): TaskBookingLabel {
  return {
    description: {
      task_definition_id: 'patrol',
      destination: task_description.places[task_description.places.length - 1],
    },
  };
}

function makeCustomComposeTaskBookingLabel(): TaskBookingLabel {
  return {
    description: {
      task_definition_id: 'custom_compose',
    },
  };
}

// A bunch of manually defined descriptions to avoid using `any`.
export interface PatrolTaskDescription {
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

interface CartCustomPickupPhase {
  activity: {
    category: string;
    description: {
      activities: [go_to_pickup: GoToPlaceActivity, pickup_cart: ZonePickupActivity];
    };
  };
}

interface CartPickupPhase {
  activity: {
    category: string;
    description: {
      activities: [go_to_pickup: GoToPlaceActivity, pickup_cart: LotPickupActivity];
    };
  };
}

interface DropoffActivity {
  category: string;
  description: {
    unix_millis_action_duration_estimate: number;
    category: string;
    description: {};
  };
}

interface OneOfWaypoint {
  waypoint: string;
}

interface GoToOneOfThePlacesActivity {
  category: string;
  description: {
    one_of: OneOfWaypoint[];
    constraints: [
      {
        category: string;
        description: string;
      },
    ];
  };
}

interface OnCancelDropoff {
  category: string;
  description: [
    go_to_one_of_the_places: GoToOneOfThePlacesActivity,
    delivery_dropoff: DropoffActivity,
  ];
}

interface DeliveryWithCancellationPhase {
  activity: {
    category: string;
    description: {
      activities: [go_to_place: GoToPlaceActivity];
    };
  };
  on_cancel: OnCancelDropoff[];
}

interface CartDropoffPhase {
  activity: {
    category: string;
    description: {
      activities: [delivery_dropoff: DropoffActivity];
    };
  };
}

export interface DeliveryCustomTaskDescription {
  category: string;
  phases: [
    pickup_phase: CartCustomPickupPhase,
    delivery_phase: DeliveryWithCancellationPhase,
    dropoff_phase: CartDropoffPhase,
  ];
}

export interface DeliveryTaskDescription {
  category: string;
  phases: [
    pickup_phase: CartPickupPhase,
    delivery_phase: DeliveryWithCancellationPhase,
    dropoff_phase: CartDropoffPhase,
  ];
}

type CustomComposeTaskDescription = string;

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
  const goToDropoff = taskDescription.phases[1].activity.description.activities[0];
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
  const goToDropoff = taskDescription.phases[1].activity.description.activities[0];
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

const isCustomTaskDescriptionValid = (taskDescription: string): boolean => {
  if (taskDescription.length === 0) {
    return false;
  }

  try {
    JSON.parse(taskDescription);
  } catch (e) {
    return false;
  }

  return true;
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

export function getShortDescription(taskRequest: TaskRequest): string | undefined {
  if (taskRequest.category === 'patrol') {
    const formattedPlaces = taskRequest.description.places.map((place: string) => `[${place}]`);
    return `[Patrol] [${taskRequest.description.rounds}] round/s, along ${formattedPlaces.join(
      ', ',
    )}`;
  }

  // This section is only valid for custom delivery types
  // FIXME: This block looks like it makes assumptions about the structure of
  // the task description in order to parse it, but it is following the
  // statically defined description (object) at the top of this file. The
  // descriptions should be replaced by a schema in general, however the better
  // approach now should be to make each task description testable and in charge
  // of their own short descriptions.
  try {
    const goToPickup: GoToPlaceActivity =
      taskRequest.description.phases[0].activity.description.activities[0];
    const pickup: LotPickupActivity =
      taskRequest.description.phases[0].activity.description.activities[1];
    const cartId = pickup.description.description.cart_id;
    const goToDropoff: GoToPlaceActivity =
      taskRequest.description.phases[1].activity.description.activities[0];

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
  } catch (e) {
    if (e instanceof TypeError) {
      console.error(`Failed to parse custom delivery: ${e.message}`);
    } else {
      console.error(
        `Failed to generate short description from task of category: ${taskRequest.category}: ${
          (e as Error).message
        }`,
      );
    }

    try {
      const descriptionString = JSON.stringify(taskRequest.description);
      console.error(descriptionString);
      return descriptionString;
    } catch (e) {
      console.error(
        `Failed to parse description of task of category: ${taskRequest.category}: ${
          (e as Error).message
        }`,
      );
      return undefined;
    }
  }
}

export function deliveryInsertPickup(
  taskDescription: DeliveryTaskDescription,
  pickupPlace: string,
  pickupLot: string,
): DeliveryTaskDescription {
  taskDescription.phases[0].activity.description.activities[0].description = pickupPlace;
  taskDescription.phases[0].activity.description.activities[1].description.description.pickup_lot =
    pickupLot;
  return taskDescription;
}

export function deliveryInsertCartId(
  taskDescription: DeliveryTaskDescription,
  cartId: string,
): DeliveryTaskDescription {
  taskDescription.phases[0].activity.description.activities[1].description.description.cart_id =
    cartId;
  return taskDescription;
}

export function deliveryInsertDropoff(
  taskDescription: DeliveryTaskDescription,
  dropoffPlace: string,
): DeliveryTaskDescription {
  taskDescription.phases[1].activity.description.activities[0].description = dropoffPlace;
  return taskDescription;
}

export function deliveryInsertOnCancel(
  taskDescription: DeliveryTaskDescription,
  onCancelPlaces: string[],
): DeliveryTaskDescription {
  const goToOneOfThePlaces: GoToOneOfThePlacesActivity = {
    category: 'go_to_place',
    description: {
      one_of: onCancelPlaces.map((placeName) => {
        return {
          waypoint: placeName,
        };
      }),
      constraints: [
        {
          category: 'prefer_same_map',
          description: '',
        },
      ],
    },
  };
  const deliveryDropoff: DropoffActivity = {
    category: 'perform_action',
    description: {
      unix_millis_action_duration_estimate: 60000,
      category: 'delivery_dropoff',
      description: {},
    },
  };
  const onCancelDropoff: OnCancelDropoff = {
    category: 'sequence',
    description: [goToOneOfThePlaces, deliveryDropoff],
  };
  taskDescription.phases[1].on_cancel = [onCancelDropoff];
  return taskDescription;
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
            let newTaskDesc = { ...taskDesc };
            newTaskDesc = deliveryInsertPickup(newTaskDesc, newValue, pickupLot);
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const place = (ev.target as HTMLInputElement).value;
            const pickupLot = pickupPoints[place] ?? '';
            let newTaskDesc = { ...taskDesc };
            newTaskDesc = deliveryInsertPickup(newTaskDesc, place, pickupLot);
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
            let newTaskDesc = { ...taskDesc };
            newTaskDesc = deliveryInsertCartId(newTaskDesc, newValue);
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            let newTaskDesc = { ...taskDesc };
            newTaskDesc = deliveryInsertCartId(newTaskDesc, (ev.target as HTMLInputElement).value);
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
          value={taskDesc.phases[1].activity.description.activities[0].description}
          onInputChange={(_ev, newValue) => {
            let newTaskDesc = { ...taskDesc };
            newTaskDesc = deliveryInsertDropoff(newTaskDesc, newValue);
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            let newTaskDesc = { ...taskDesc };
            newTaskDesc = deliveryInsertDropoff(newTaskDesc, (ev.target as HTMLInputElement).value);
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
                  taskDesc.phases[1].activity.description.activities[0].description,
                )
              }
            />
          )}
        />
      </Grid>
    </Grid>
  );
}

export function deliveryCustomInsertPickup(
  taskDescription: DeliveryCustomTaskDescription,
  pickupPlace: string,
  pickupZone: string,
): DeliveryCustomTaskDescription {
  taskDescription.phases[0].activity.description.activities[0].description = pickupPlace;
  taskDescription.phases[0].activity.description.activities[1].description.description.pickup_zone =
    pickupZone;
  return taskDescription;
}

export function deliveryCustomInsertCartId(
  taskDescription: DeliveryCustomTaskDescription,
  cartId: string,
): DeliveryCustomTaskDescription {
  taskDescription.phases[0].activity.description.activities[1].description.description.cart_id =
    cartId;
  return taskDescription;
}

export function deliveryCustomInsertDropoff(
  taskDescription: DeliveryCustomTaskDescription,
  dropoffPlace: string,
): DeliveryCustomTaskDescription {
  taskDescription.phases[1].activity.description.activities[0].description = dropoffPlace;
  return taskDescription;
}

export function deliveryCustomInsertOnCancel(
  taskDescription: DeliveryCustomTaskDescription,
  onCancelPlaces: string[],
): DeliveryCustomTaskDescription {
  const goToOneOfThePlaces: GoToOneOfThePlacesActivity = {
    category: 'go_to_place',
    description: {
      one_of: onCancelPlaces.map((placeName) => {
        return {
          waypoint: placeName,
        };
      }),
      constraints: [
        {
          category: 'prefer_same_map',
          description: '',
        },
      ],
    },
  };
  const deliveryDropoff: DropoffActivity = {
    category: 'perform_action',
    description: {
      unix_millis_action_duration_estimate: 60000,
      category: 'delivery_dropoff',
      description: {},
    },
  };
  const onCancelDropoff: OnCancelDropoff = {
    category: 'sequence',
    description: [goToOneOfThePlaces, deliveryDropoff],
  };
  taskDescription.phases[1].on_cancel = [onCancelDropoff];
  return taskDescription;
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
            let newTaskDesc = { ...taskDesc };
            newTaskDesc = deliveryCustomInsertPickup(newTaskDesc, newValue, newValue);
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const zone = (ev.target as HTMLInputElement).value;
            let newTaskDesc = { ...taskDesc };
            newTaskDesc = deliveryCustomInsertPickup(newTaskDesc, zone, zone);
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
            let newTaskDesc = { ...taskDesc };
            newTaskDesc = deliveryCustomInsertCartId(newTaskDesc, newValue);
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            let newTaskDesc = { ...taskDesc };
            newTaskDesc = deliveryCustomInsertCartId(
              newTaskDesc,
              (ev.target as HTMLInputElement).value,
            );
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
          value={taskDesc.phases[1].activity.description.activities[0].description}
          onInputChange={(_ev, newValue) => {
            let newTaskDesc = { ...taskDesc };
            newTaskDesc = deliveryCustomInsertDropoff(newTaskDesc, newValue);
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            let newTaskDesc = { ...taskDesc };
            newTaskDesc = deliveryCustomInsertDropoff(
              newTaskDesc,
              (ev.target as HTMLInputElement).value,
            );
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
                  taskDesc.phases[1].activity.description.activities[0].description,
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
  allowSubmit(isPatrolTaskDescriptionValid(taskDesc));

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

interface CustomComposeTaskFormProps {
  taskDesc: CustomComposeTaskDescription;
  onChange(customComposeTaskDescription: CustomComposeTaskDescription): void;
  allowSubmit(allow: boolean): void;
}

function CustomComposeTaskForm({ taskDesc, onChange, allowSubmit }: CustomComposeTaskFormProps) {
  const theme = useTheme();
  const onInputChange = (desc: CustomComposeTaskDescription) => {
    allowSubmit(isCustomTaskDescriptionValid(desc));
    onChange(desc);
  };

  return (
    <Grid container spacing={theme.spacing(2)}>
      <Grid item xs={12}>
        <TextField
          label="Multiline"
          multiline
          rows={8}
          value={taskDesc}
          fullWidth
          onChange={(ev) => {
            onInputChange(ev.target.value);
          }}
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

export function defaultDeliveryTaskDescription(): DeliveryTaskDescription {
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
            ],
          },
        },
      },
      {
        activity: {
          category: 'sequence',
          description: {
            activities: [
              {
                category: 'go_to_place',
                description: '',
              },
            ],
          },
        },
        on_cancel: [],
      },
      {
        activity: {
          category: 'sequence',
          description: {
            activities: [
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

export function defaultDeliveryCustomTaskDescription(
  taskCategory: string,
): DeliveryCustomTaskDescription {
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
            ],
          },
        },
      },
      {
        activity: {
          category: 'sequence',
          description: {
            activities: [
              {
                category: 'go_to_place',
                description: '',
              },
            ],
          },
        },
        on_cancel: [],
      },
      {
        activity: {
          category: 'sequence',
          description: {
            activities: [
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

export function defaultPatrolTask(): PatrolTaskDescription {
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
    task_definition_id: DefaultTaskDefinitionId,
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
  emergencyLots?: string[];
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
  /* eslint-disable @typescript-eslint/no-unused-vars */
  cleaningZones = [],
  /* eslint-disable @typescript-eslint/no-unused-vars */
  patrolWaypoints = [],
  pickupZones = [],
  cartIds = [],
  emergencyLots = [],
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
    SupportedTaskDefinitions[0].task_definition_id,
  );
  const [taskRequest, setTaskRequest] = React.useState<TaskRequest>(
    () => requestTask ?? defaultTask(),
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
    if (taskRequest.category === 'patrol') {
      return (
        <PatrolTaskForm
          taskDesc={taskRequest.description as PatrolTaskDescription}
          patrolWaypoints={patrolWaypoints}
          onChange={(desc) => handleTaskDescriptionChange('patrol', desc)}
          allowSubmit={allowSubmit}
        />
      );
    } else if (taskRequest.category === 'custom_compose') {
      return (
        <CustomComposeTaskForm
          taskDesc={taskRequest.description as CustomComposeTaskDescription}
          onChange={(desc) => handleCustomComposeTaskDescriptionChange(desc)}
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
      const category = newType === 'patrol' ? 'patrol' : 'compose';
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

    // if (
    //   taskDefinition === 'delivery_pickup' ||
    //   taskDefinition === 'delivery_sequential_lot_pickup' ||
    //   taskDefinition === 'delivery_area_pickup'
    // ) {
    //   const goToOneOfThePlaces: GoToOneOfThePlacesActivity = {
    //     category: 'go_to_place',
    //     description: {
    //       one_of: emergencyLots.map((placeName) => {
    //         return {
    //           waypoint: placeName,
    //         };
    //       }),
    //       constraints: [
    //         {
    //           category: 'prefer_same_map',
    //           description: '',
    //         },
    //       ],
    //     },
    //   };

    //   // FIXME: there should not be any statically defined duration estimates as
    //   // it makes assumptions of the deployments.
    //   const deliveryDropoff: DropoffActivity = {
    //     category: 'perform_action',
    //     description: {
    //       unix_millis_action_duration_estimate: 60000,
    //       category: 'delivery_dropoff',
    //       description: {},
    //     },
    //   };
    //   const onCancelDropoff: OnCancelDropoff = {
    //     category: 'sequence',
    //     description: [goToOneOfThePlaces, deliveryDropoff],
    //   };
    //   request.description.phases[1].on_cancel = [onCancelDropoff];
    // } else if (taskDefinition === 'custom_compose') {
    //   try {
    //     const obj = JSON.parse(request.description);
    //     request.category = 'compose';
    //     request.description = obj;
    //   } catch (e) {
    //     console.error('Invalid custom compose task description');
    //     onFail && onFail(e as Error, [request]);
    //     return;
    //   }
    // }

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

      setTaskRequest(defaultTask());
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
                      {SupportedTaskDefinitions.map((taskDefinition) => (
                        <MenuItem value={taskDefinition.task_definition_id}>
                          {taskDefinition.task_display_name}
                        </MenuItem>
                      ))}
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
