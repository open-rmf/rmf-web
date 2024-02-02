import { isNonEmptyString, isPositiveNumber } from './utils';
import { Autocomplete, Grid, TextField, useTheme } from '@mui/material';
import { PositiveIntField } from '../../form-inputs';
import React from 'react';

interface TaskPlace {
  place: string;
  handler: string;
  payload: {
    sku: string;
    quantity: number;
  };
}

export interface DeliveryTaskDescription {
  category: string;
  phases: [
    deliveryPhase: {
      activity: {
        category: string;
        description: {
          activities: [
            deliveryActivity: {
              pickup: TaskPlace;
              dropoff: TaskPlace;
            },
          ];
        };
      };
    },
  ];
}

function isTaskPlaceValid(place: TaskPlace): boolean {
  return (
    isNonEmptyString(place.place) &&
    isNonEmptyString(place.handler) &&
    isNonEmptyString(place.payload.sku) &&
    isPositiveNumber(place.payload.quantity)
  );
}

const isDeliveryTaskDescriptionValid = (taskDescription: DeliveryTaskDescription): boolean => {
  const deliveryActivity = taskDescription.phases[0].activity.description.activities[0];
  return isTaskPlaceValid(deliveryActivity.pickup) && isTaskPlaceValid(deliveryActivity.dropoff);
};

export function makeDefaultDeliveryTaskDescription(): DeliveryTaskDescription {
  return {
    category: 'delivery',
    phases: [
      {
        activity: {
          category: 'sequence',
          description: {
            activities: [
              {
                pickup: {
                  place: '',
                  handler: '',
                  payload: {
                    sku: '',
                    quantity: 1,
                  },
                },
                dropoff: {
                  place: '',
                  handler: '',
                  payload: {
                    sku: '',
                    quantity: 1,
                  },
                },
              },
            ],
          },
        },
      },
    ],
  };
}

export interface DeliveryTaskFormProps {
  taskDesc: DeliveryTaskDescription;
  pickupPoints: Record<string, string>;
  dropoffPoints: Record<string, string>;
  onChange(taskDesc: DeliveryTaskDescription): void;
  allowSubmit(allow: boolean): void;
}

export function DeliveryTaskForm({
  taskDesc,
  pickupPoints = {},
  dropoffPoints = {},
  onChange,
  allowSubmit,
}: DeliveryTaskFormProps) {
  const theme = useTheme();
  const onInputChange = (desc: DeliveryTaskDescription) => {
    allowSubmit(isDeliveryTaskDescriptionValid(desc));
    onChange(desc);
  };

  return (
    <Grid container spacing={theme.spacing(2)} justifyContent="center" alignItems="center">
      <Grid item xs={6}>
        <Autocomplete
          id="pickup-location"
          freeSolo
          fullWidth
          options={Object.keys(pickupPoints)}
          value={taskDesc.phases[0].activity.description.activities[0].pickup.place}
          onChange={(_ev, newValue) => {
            const place = newValue ?? '';
            const handler =
              newValue !== null && pickupPoints[newValue] ? pickupPoints[newValue] : '';
            taskDesc.phases[0].activity.description.activities[0].pickup.place = place;
            taskDesc.phases[0].activity.description.activities[0].pickup.handler = handler;
            onInputChange(taskDesc);
          }}
          onBlur={(ev) => {
            if (!pickupPoints[(ev.target as HTMLInputElement).value]) {
              return;
            }
            taskDesc.phases[0].activity.description.activities[0].pickup.place = (
              ev.target as HTMLInputElement
            ).value;
            taskDesc.phases[0].activity.description.activities[0].pickup.handler =
              pickupPoints[(ev.target as HTMLInputElement).value];
            onInputChange(taskDesc);
          }}
          renderInput={(params) => (
            <TextField {...params} label="Pickup Location" required={true} />
          )}
        />
      </Grid>
      <Grid item xs={4}>
        <TextField
          id="pickup_sku"
          fullWidth
          label="Pickup SKU"
          value={taskDesc.phases[0].activity.description.activities[0].pickup.payload.sku}
          required
          onChange={(ev) => {
            taskDesc.phases[0].activity.description.activities[0].pickup.payload.sku =
              ev.target.value;
            onInputChange(taskDesc);
          }}
        />
      </Grid>
      <Grid item xs={2}>
        <PositiveIntField
          id="pickup_quantity"
          label="Quantity"
          value={taskDesc.phases[0].activity.description.activities[0].pickup.payload.quantity}
          onChange={(_ev, val) => {
            taskDesc.phases[0].activity.description.activities[0].pickup.payload.quantity = val;
            onInputChange(taskDesc);
          }}
        />
      </Grid>
      <Grid item xs={6}>
        <Autocomplete
          id="dropoff-location"
          freeSolo
          fullWidth
          options={Object.keys(dropoffPoints)}
          value={taskDesc.phases[0].activity.description.activities[0].dropoff.place}
          onChange={(_ev, newValue) => {
            const place = newValue ?? '';
            const handler =
              newValue !== null && dropoffPoints[newValue] ? dropoffPoints[newValue] : '';
            taskDesc.phases[0].activity.description.activities[0].dropoff.place = place;
            taskDesc.phases[0].activity.description.activities[0].dropoff.handler = handler;
            onInputChange(taskDesc);
          }}
          onBlur={(ev) => {
            if (!dropoffPoints[(ev.target as HTMLInputElement).value]) {
              return;
            }
            taskDesc.phases[0].activity.description.activities[0].dropoff.place = (
              ev.target as HTMLInputElement
            ).value;
            taskDesc.phases[0].activity.description.activities[0].dropoff.handler =
              dropoffPoints[(ev.target as HTMLInputElement).value];
            onInputChange(taskDesc);
          }}
          renderInput={(params) => (
            <TextField {...params} label="Dropoff Location" required={true} />
          )}
        />
      </Grid>
      <Grid item xs={4}>
        <TextField
          id="dropoff_sku"
          fullWidth
          label="Dropoff SKU"
          value={taskDesc.phases[0].activity.description.activities[0].dropoff.payload.sku}
          required
          onChange={(ev) => {
            taskDesc.phases[0].activity.description.activities[0].dropoff.payload.sku =
              ev.target.value;
            onInputChange(taskDesc);
          }}
        />
      </Grid>
      <Grid item xs={2}>
        <PositiveIntField
          id="dropoff_quantity"
          label="Quantity"
          value={taskDesc.phases[0].activity.description.activities[0].dropoff.payload.quantity}
          onChange={(_ev, val) => {
            taskDesc.phases[0].activity.description.activities[0].dropoff.payload.quantity = val;
            onInputChange(taskDesc);
          }}
        />
      </Grid>
    </Grid>
  );
}
