import { Autocomplete, Grid, TextField, useMediaQuery, useTheme } from '@mui/material';
import React from 'react';
import { isNonEmptyString } from './utils';

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
  onChange(taskDesc: DeliveryTaskDescription): void;
  allowSubmit(allow: boolean): void;
}

export function DeliveryTaskForm({
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

export function DeliveryCustomTaskForm({
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
