import { Autocomplete, Grid, TextField, useMediaQuery, useTheme } from '@mui/material';
import React from 'react';
import type { TaskBookingLabel } from 'api-client';
import { TaskDefinition } from '../create-task';

export const DeliveryPickupTaskDefinition: TaskDefinition = {
  taskDefinitionId: 'delivery_pickup',
  taskDisplayName: 'Delivery - 1:1',
  requestCategory: 'compose',
};

export const DeliverySequentialLotPickupTaskDefinition: TaskDefinition = {
  taskDefinitionId: 'delivery_sequential_lot_pickup',
  taskDisplayName: 'Delivery - Sequential lot pick up',
  requestCategory: 'compose',
};

export const DeliveryAreaPickupTaskDefinition: TaskDefinition = {
  taskDefinitionId: 'delivery_area_pickup',
  taskDisplayName: 'Delivery - Area pick up',
  requestCategory: 'compose',
};

export const DoubleComposeDeliveryTaskDefinition: TaskDefinition = {
  taskDefinitionId: 'double-compose-delivery',
  taskDisplayName: 'Double Compose Delivery',
  requestCategory: 'compose',
};

export interface LotPickupActivity {
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

export interface GoToPlaceActivity {
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

export interface CartPickupPhase {
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

export interface DeliveryWithCancellationPhase {
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

export interface DeliveryPickupTaskDescription {
  category: string;
  phases: [
    pickup_phase: CartPickupPhase,
    delivery_phase: DeliveryWithCancellationPhase,
    dropoff_phase: CartDropoffPhase,
  ];
}

export interface DoubleComposeDeliveryTaskDescription {
  category: string;
  phases: [
    first_pickup_phase: CartPickupPhase,
    first_delivery_phase: DeliveryWithCancellationPhase,
    first_dropoff_phase: CartDropoffPhase,
    second_pickup_phase: CartPickupPhase,
    second_delivery_phase: DeliveryWithCancellationPhase,
    second_dropoff_phase: CartDropoffPhase,
  ];
}

export function makeDeliveryPickupTaskBookingLabel(
  task_description: DeliveryPickupTaskDescription,
): TaskBookingLabel {
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

export function makeDeliveryCustomTaskBookingLabel(
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

export function makeDoubleComposeDeliveryTaskBookingLabel(
  task_description: DoubleComposeDeliveryTaskDescription,
): TaskBookingLabel {
  const firstPickupDescription =
    task_description.phases[0].activity.description.activities[1].description.description;
  const secondPickupDescription =
    task_description.phases[3].activity.description.activities[1].description.description;

  const pickups = `${firstPickupDescription.pickup_lot}(1), ${secondPickupDescription.pickup_lot}(2)`;
  const dropoffs = `${task_description.phases[1].activity.description.activities[0].description}(1), ${task_description.phases[4].activity.description.activities[0].description}(2)`;
  const cartIds = `${firstPickupDescription.pickup_lot}(1), ${secondPickupDescription.cart_id}(2)`;

  return {
    description: {
      task_definition_id: task_description.category,
      pickup: pickups,
      destination: dropoffs,
      cart_id: cartIds,
    },
  };
}

export function makeDeliveryPickupTaskShortDescription(
  desc: DeliveryPickupTaskDescription,
  taskDisplayName: string | undefined,
): string {
  try {
    const goToPickup: GoToPlaceActivity = desc.phases[0].activity.description.activities[0];
    const pickup: LotPickupActivity = desc.phases[0].activity.description.activities[1];
    const cartId = pickup.description.description.cart_id;
    const goToDropoff: GoToPlaceActivity = desc.phases[1].activity.description.activities[0];

    return `[${
      taskDisplayName ?? DeliveryPickupTaskDefinition.taskDisplayName
    }] payload [${cartId}] from [${goToPickup.description}] to [${goToDropoff.description}]`;
  } catch (e) {
    try {
      const descriptionString = JSON.stringify(desc);
      console.error(descriptionString);
      return descriptionString;
    } catch (e) {
      console.error(
        `Failed to parse task description of delivery pickup task: ${(e as Error).message}`,
      );
    }
  }

  return '[Unknown] delivery pickup task';
}

export function makeDoubleComposeDeliveryTaskShortDescription(
  desc: DoubleComposeDeliveryTaskDescription,
  taskDisplayName: string | undefined,
): string {
  try {
    const firstGoToPickup: GoToPlaceActivity = desc.phases[0].activity.description.activities[0];
    const firstPickup: LotPickupActivity = desc.phases[0].activity.description.activities[1];
    const firstCartId = firstPickup.description.description.cart_id;
    const firstGoToDropoff: GoToPlaceActivity = desc.phases[1].activity.description.activities[0];

    const secondGoToPickup: GoToPlaceActivity = desc.phases[3].activity.description.activities[0];
    const secondPickup: LotPickupActivity = desc.phases[3].activity.description.activities[1];
    const secondCartId = secondPickup.description.description.cart_id;
    const secondGoToDropoff: GoToPlaceActivity = desc.phases[4].activity.description.activities[0];

    return `[${
      taskDisplayName ?? DeliveryPickupTaskDefinition.taskDisplayName
    }] payload [${firstCartId}] from [${firstGoToPickup.description}] to [${
      firstGoToDropoff.description
    }], then payload [${secondCartId}] from [${secondGoToPickup.description}] to [${
      secondGoToDropoff.description
    }]`;
  } catch (e) {
    try {
      const descriptionString = JSON.stringify(desc);
      console.error(descriptionString);
      return descriptionString;
    } catch (e) {
      console.error(
        `Failed to parse task description of double compose delivery task: ${(e as Error).message}`,
      );
    }
  }

  return '[Unknown] double compose delivery task';
}

export function makeDeliveryCustomTaskShortDescription(
  desc: DeliveryCustomTaskDescription,
  taskDisplayName: string | undefined,
): string {
  try {
    const goToPickup: GoToPlaceActivity = desc.phases[0].activity.description.activities[0];
    const pickup: ZonePickupActivity = desc.phases[0].activity.description.activities[1];
    const cartId = pickup.description.description.cart_id;
    const goToDropoff: GoToPlaceActivity = desc.phases[1].activity.description.activities[0];

    switch (desc.category) {
      case DeliverySequentialLotPickupTaskDefinition.taskDefinitionId: {
        return `[${
          taskDisplayName ?? DeliverySequentialLotPickupTaskDefinition.taskDisplayName
        }] payload [${cartId}] from [${goToPickup.description}] to [${goToDropoff.description}]`;
      }
      case DeliveryAreaPickupTaskDefinition.taskDefinitionId: {
        return `[${
          taskDisplayName ?? DeliveryAreaPickupTaskDefinition.taskDisplayName
        }] payload [${cartId}] from [${goToPickup.description}] to [${goToDropoff.description}]`;
      }
      default:
        return `[Unknown] type "${desc.category}"`;
    }
  } catch (e) {
    try {
      const descriptionString = JSON.stringify(desc);
      console.error(descriptionString);
      return descriptionString;
    } catch (e) {
      console.error(
        `Failed to parse task description of delivery pickup task: ${(e as Error).message}`,
      );
    }
  }

  return '[Unknown] delivery pickup task';
}

const isDeliveryPickupTaskDescriptionValid = (
  taskDescription: DeliveryPickupTaskDescription,
  pickupPoints: Record<string, string>,
  dropoffPoints: Record<string, string>,
): boolean => {
  const goToPickup = taskDescription.phases[0].activity.description.activities[0];
  const pickup = taskDescription.phases[0].activity.description.activities[1];
  const goToDropoff = taskDescription.phases[1].activity.description.activities[0];
  return (
    goToPickup.description.length > 0 &&
    Object.keys(pickupPoints).includes(goToPickup.description) &&
    pickupPoints[goToPickup.description] === pickup.description.description.pickup_lot &&
    pickup.description.description.cart_id.length > 0 &&
    goToDropoff.description.length > 0 &&
    Object.keys(dropoffPoints).includes(goToDropoff.description)
  );
};

const isDoubleComposeDeliveryTaskDescriptionValid = (
  taskDescription: DoubleComposeDeliveryTaskDescription,
  pickupPoints: Record<string, string>,
  dropoffPoints: Record<string, string>,
): boolean => {
  const firstGoToPickup = taskDescription.phases[0].activity.description.activities[0];
  const firstPickup = taskDescription.phases[0].activity.description.activities[1];
  const firstGoToDropoff = taskDescription.phases[1].activity.description.activities[0];

  const secondGoToPickup = taskDescription.phases[3].activity.description.activities[0];
  const secondPickup = taskDescription.phases[3].activity.description.activities[1];
  const secondGoToDropoff = taskDescription.phases[4].activity.description.activities[0];

  return (
    firstGoToPickup.description.length > 0 &&
    Object.keys(pickupPoints).includes(firstGoToPickup.description) &&
    pickupPoints[firstGoToPickup.description] === firstPickup.description.description.pickup_lot &&
    firstPickup.description.description.cart_id.length > 0 &&
    firstGoToDropoff.description.length > 0 &&
    Object.keys(dropoffPoints).includes(firstGoToDropoff.description) &&
    secondGoToPickup.description.length > 0 &&
    Object.keys(pickupPoints).includes(secondGoToPickup.description) &&
    pickupPoints[secondGoToPickup.description] ===
      secondPickup.description.description.pickup_lot &&
    secondPickup.description.description.cart_id.length > 0 &&
    secondGoToDropoff.description.length > 0 &&
    Object.keys(dropoffPoints).includes(secondGoToDropoff.description)
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
    goToPickup.description.length > 0 &&
    pickup.description.description.pickup_zone.length > 0 &&
    pickupZones.includes(pickup.description.description.pickup_zone) &&
    pickup.description.description.cart_id.length > 0 &&
    goToDropoff.description.length > 0 &&
    dropoffPoints.includes(goToDropoff.description)
  );
};

export function cartPickupPhaseInsertPickup(
  phase: CartPickupPhase,
  pickupPlace: string,
  pickupLot: string,
): CartPickupPhase {
  phase.activity.description.activities[0].description = pickupPlace;
  phase.activity.description.activities[1].description.description.pickup_lot = pickupLot;
  return phase;
}

export function cartPickupPhaseInsertCartId(
  phase: CartPickupPhase,
  cartId: string,
): CartPickupPhase {
  phase.activity.description.activities[1].description.description.cart_id = cartId;
  return phase;
}

export function deliveryPhaseInsertDropoff(
  phase: DeliveryWithCancellationPhase,
  dropoffPlace: string,
): DeliveryWithCancellationPhase {
  phase.activity.description.activities[0].description = dropoffPlace;
  return phase;
}

export function deliveryPhaseInsertOnCancel(
  phase: DeliveryWithCancellationPhase,
  onCancelPlaces: string[],
): DeliveryWithCancellationPhase {
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
  phase.on_cancel = [onCancelDropoff];
  return phase;
}

interface DeliveryPickupTaskFormProps {
  taskDesc: DeliveryPickupTaskDescription;
  pickupPoints: Record<string, string>;
  cartIds: string[];
  dropoffPoints: Record<string, string>;
  onChange(taskDesc: DeliveryPickupTaskDescription): void;
  onValidate(valid: boolean): void;
}

export function DeliveryPickupTaskForm({
  taskDesc,
  pickupPoints = {},
  cartIds = [],
  dropoffPoints = {},
  onChange,
  onValidate,
}: DeliveryPickupTaskFormProps): React.JSX.Element {
  const theme = useTheme();
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  const onInputChange = (desc: DeliveryPickupTaskDescription) => {
    onValidate(isDeliveryPickupTaskDescriptionValid(desc, pickupPoints, dropoffPoints));
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
            newTaskDesc.phases[0] = cartPickupPhaseInsertPickup(
              newTaskDesc.phases[0],
              newValue,
              pickupLot,
            );
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const place = (ev.target as HTMLInputElement).value;
            const pickupLot = pickupPoints[place] ?? '';
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0] = cartPickupPhaseInsertPickup(
              newTaskDesc.phases[0],
              place,
              pickupLot,
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
            newTaskDesc.phases[0] = cartPickupPhaseInsertCartId(newTaskDesc.phases[0], newValue);
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0] = cartPickupPhaseInsertCartId(
              newTaskDesc.phases[0],
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
          options={Object.keys(dropoffPoints).sort()}
          value={taskDesc.phases[1].activity.description.activities[0].description}
          onInputChange={(_ev, newValue) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[1] = deliveryPhaseInsertDropoff(newTaskDesc.phases[1], newValue);
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[1] = deliveryPhaseInsertDropoff(
              newTaskDesc.phases[1],
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

interface DoubleComposeDeliveryTaskFormProps {
  taskDesc: DoubleComposeDeliveryTaskDescription;
  pickupPoints: Record<string, string>;
  cartIds: string[];
  dropoffPoints: Record<string, string>;
  onChange(taskDesc: DoubleComposeDeliveryTaskDescription): void;
  onValidate(valid: boolean): void;
}

export function DoubleComposeDeliveryTaskForm({
  taskDesc,
  pickupPoints = {},
  cartIds = [],
  dropoffPoints = {},
  onChange,
  onValidate,
}: DoubleComposeDeliveryTaskFormProps): React.JSX.Element {
  const theme = useTheme();
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  const onInputChange = (desc: DoubleComposeDeliveryTaskDescription) => {
    onValidate(isDoubleComposeDeliveryTaskDescriptionValid(desc, pickupPoints, dropoffPoints));
    onChange(desc);
  };

  return (
    <Grid container spacing={theme.spacing(2)} justifyContent="left" alignItems="center">
      <Grid item xs={5}>
        <Autocomplete
          id="first-pickup-location"
          freeSolo
          fullWidth
          options={Object.keys(pickupPoints).sort()}
          value={taskDesc.phases[0].activity.description.activities[0].description}
          onInputChange={(_ev, newValue) => {
            const pickupLot = pickupPoints[newValue] ?? '';
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0] = cartPickupPhaseInsertPickup(
              newTaskDesc.phases[0],
              newValue,
              pickupLot,
            );
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const place = (ev.target as HTMLInputElement).value;
            const pickupLot = pickupPoints[place] ?? '';
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0] = cartPickupPhaseInsertPickup(
              newTaskDesc.phases[0],
              place,
              pickupLot,
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
              label="Cart pickup (1)"
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
      <Grid item xs={5}>
        <Autocomplete
          id="first-dropoff-location"
          freeSolo
          fullWidth
          options={Object.keys(dropoffPoints).sort()}
          value={taskDesc.phases[1].activity.description.activities[0].description}
          onInputChange={(_ev, newValue) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[1] = deliveryPhaseInsertDropoff(newTaskDesc.phases[1], newValue);
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[1] = deliveryPhaseInsertDropoff(
              newTaskDesc.phases[1],
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
              label="Cart dropoff (1)"
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
      <Grid item xs={2}>
        <Autocomplete
          id="first-cart-id"
          freeSolo
          fullWidth
          options={cartIds}
          value={
            taskDesc.phases[0].activity.description.activities[1].description.description.cart_id
          }
          getOptionLabel={(option) => option}
          onInputChange={(_ev, newValue) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0] = cartPickupPhaseInsertCartId(newTaskDesc.phases[0], newValue);
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0] = cartPickupPhaseInsertCartId(
              newTaskDesc.phases[0],
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
              label="Cart ID (1)"
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
      <Grid item xs={5}>
        <Autocomplete
          id="second-pickup-location"
          freeSolo
          fullWidth
          options={Object.keys(pickupPoints).sort()}
          value={taskDesc.phases[3].activity.description.activities[0].description}
          onInputChange={(_ev, newValue) => {
            const pickupLot = pickupPoints[newValue] ?? '';
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[3] = cartPickupPhaseInsertPickup(
              newTaskDesc.phases[3],
              newValue,
              pickupLot,
            );
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const place = (ev.target as HTMLInputElement).value;
            const pickupLot = pickupPoints[place] ?? '';
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[3] = cartPickupPhaseInsertPickup(
              newTaskDesc.phases[3],
              place,
              pickupLot,
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
              label="Cart pickup (2)"
              required
              InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 14 : 20 } }}
              error={
                !Object.keys(pickupPoints).includes(
                  taskDesc.phases[3].activity.description.activities[0].description,
                )
              }
            />
          )}
        />
      </Grid>
      <Grid item xs={5}>
        <Autocomplete
          id="second-dropoff-location"
          freeSolo
          fullWidth
          options={Object.keys(dropoffPoints).sort()}
          value={taskDesc.phases[4].activity.description.activities[0].description}
          onInputChange={(_ev, newValue) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[4] = deliveryPhaseInsertDropoff(newTaskDesc.phases[4], newValue);
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[4] = deliveryPhaseInsertDropoff(
              newTaskDesc.phases[4],
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
              label="Cart dropoff (2)"
              required
              InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 14 : 20 } }}
              error={
                !Object.keys(dropoffPoints).includes(
                  taskDesc.phases[4].activity.description.activities[0].description,
                )
              }
            />
          )}
        />
      </Grid>
      <Grid item xs={2}>
        <Autocomplete
          id="second-cart-id"
          freeSolo
          fullWidth
          options={cartIds}
          value={
            taskDesc.phases[3].activity.description.activities[1].description.description.cart_id
          }
          getOptionLabel={(option) => option}
          onInputChange={(_ev, newValue) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[3] = cartPickupPhaseInsertCartId(newTaskDesc.phases[3], newValue);
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[3] = cartPickupPhaseInsertCartId(
              newTaskDesc.phases[3],
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
              label="Cart ID (2)"
              required
              InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 14 : 20 } }}
              error={
                taskDesc.phases[3].activity.description.activities[1].description.description
                  .cart_id.length === 0
              }
            />
          )}
        />
      </Grid>
    </Grid>
  );
}

export function cartCustomPickupPhaseInsertPickup(
  phase: CartCustomPickupPhase,
  pickupPlace: string,
  pickupZone: string,
): CartCustomPickupPhase {
  phase.activity.description.activities[0].description = pickupPlace;
  phase.activity.description.activities[1].description.description.pickup_zone = pickupZone;
  return phase;
}

export function cartCustomPickupPhaseInsertCartId(
  phase: CartCustomPickupPhase,
  cartId: string,
): CartCustomPickupPhase {
  phase.activity.description.activities[1].description.description.cart_id = cartId;
  return phase;
}

export interface DeliveryCustomProps {
  taskDesc: DeliveryCustomTaskDescription;
  pickupZones: string[];
  cartIds: string[];
  dropoffPoints: string[];
  onChange(taskDesc: DeliveryCustomTaskDescription): void;
  onValidate(valid: boolean): void;
}

export function DeliveryCustomTaskForm({
  taskDesc,
  pickupZones = [],
  cartIds = [],
  dropoffPoints = [],
  onChange,
  onValidate,
}: DeliveryCustomProps): React.JSX.Element {
  const theme = useTheme();
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  const onInputChange = (desc: DeliveryCustomTaskDescription) => {
    onValidate(isDeliveryCustomTaskDescriptionValid(desc, pickupZones, dropoffPoints));
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
            newTaskDesc.phases[0] = cartCustomPickupPhaseInsertPickup(
              newTaskDesc.phases[0],
              newValue,
              newValue,
            );
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const zone = (ev.target as HTMLInputElement).value;
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0] = cartCustomPickupPhaseInsertPickup(
              newTaskDesc.phases[0],
              zone,
              zone,
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
            newTaskDesc.phases[0] = cartCustomPickupPhaseInsertCartId(
              newTaskDesc.phases[0],
              newValue,
            );
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[0] = cartCustomPickupPhaseInsertCartId(
              newTaskDesc.phases[0],
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
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[1] = deliveryPhaseInsertDropoff(newTaskDesc.phases[1], newValue);
            onInputChange(newTaskDesc);
          }}
          onBlur={(ev) => {
            const newTaskDesc = { ...taskDesc };
            newTaskDesc.phases[1] = deliveryPhaseInsertDropoff(
              newTaskDesc.phases[1],
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

export function makeDefaultDeliveryPickupTaskDescription(): DeliveryPickupTaskDescription {
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

export function makeDefaultDoubleComposeDeliveryTaskDescription(): DoubleComposeDeliveryTaskDescription {
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

export function makeDefaultDeliveryCustomTaskDescription(
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
