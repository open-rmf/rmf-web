import { Autocomplete, Grid, TextField, useTheme } from '@mui/material';
import React from 'react';

import { PositiveIntField } from '../../form-inputs';
import { TaskBookingLabels } from '../booking-label';
import { TaskDefinition } from '../task-form';
import { isNonEmptyString, isPositiveNumber } from './utils';

export const DeliveryTaskDefinition: TaskDefinition = {
  taskDefinitionId: 'delivery',
  taskDisplayName: 'Delivery',
  requestCategory: 'delivery',
  scheduleEventColor: undefined,
};

interface TaskPlace {
  place: string;
  handler: string;
  payload: {
    sku: string;
    quantity: number;
  };
}

export interface DeliveryTaskDescription {
  pickup: TaskPlace;
  dropoff: TaskPlace;
}

export function makeDeliveryTaskBookingLabel(
  task_description: DeliveryTaskDescription,
): TaskBookingLabels {
  return {
    task_definition_id: DeliveryTaskDefinition.taskDefinitionId,
    pickup: task_description.pickup.place,
    destination: task_description.dropoff.place,
    cart_id: task_description.pickup.payload.sku,
  };
}

function isTaskPlaceValid(place: TaskPlace): boolean {
  return (
    isNonEmptyString(place.place) &&
    isNonEmptyString(place.handler) &&
    isNonEmptyString(place.payload.sku) &&
    isPositiveNumber(place.payload.quantity)
  );
}

function isDeliveryTaskDescriptionValid(taskDescription: DeliveryTaskDescription): boolean {
  return isTaskPlaceValid(taskDescription.pickup) && isTaskPlaceValid(taskDescription.dropoff);
}

export function makeDefaultDeliveryTaskDescription(): DeliveryTaskDescription {
  return {
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
  };
}

export function makeDeliveryTaskShortDescription(
  desc: DeliveryTaskDescription,
  displayName?: string,
): string {
  return `[${displayName ?? DeliveryTaskDefinition.taskDisplayName}] Pickup [${
    desc.pickup.payload.sku
  }] from [${desc.pickup.place}], dropoff [${desc.dropoff.payload.sku}] at [${desc.dropoff.place}]`;
}

export interface DeliveryTaskFormProps {
  taskDesc: DeliveryTaskDescription;
  pickupPoints: Record<string, string>;
  dropoffPoints: Record<string, string>;
  onChange(taskDesc: DeliveryTaskDescription): void;
  onValidate(valid: boolean): void;
}

export function DeliveryTaskForm({
  taskDesc,
  pickupPoints = {},
  dropoffPoints = {},
  onChange,
  onValidate,
}: DeliveryTaskFormProps): React.JSX.Element {
  const theme = useTheme();
  const onInputChange = (desc: DeliveryTaskDescription) => {
    onValidate(isDeliveryTaskDescriptionValid(desc));
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
          value={taskDesc.pickup.place}
          onChange={(_ev, newValue) => {
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
          onBlur={(ev) =>
            pickupPoints[(ev.target as HTMLInputElement).value] &&
            onInputChange({
              ...taskDesc,
              pickup: {
                ...taskDesc.pickup,
                place: (ev.target as HTMLInputElement).value,
                handler: pickupPoints[(ev.target as HTMLInputElement).value],
              },
            })
          }
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
          value={taskDesc.pickup.payload.sku}
          required
          onChange={(ev) => {
            onInputChange({
              ...taskDesc,
              pickup: {
                ...taskDesc.pickup,
                payload: {
                  ...taskDesc.pickup.payload,
                  sku: ev.target.value,
                },
              },
            });
          }}
        />
      </Grid>
      <Grid item xs={2}>
        <PositiveIntField
          id="pickup_quantity"
          label="Quantity"
          value={taskDesc.pickup.payload.quantity}
          onChange={(_ev, val) => {
            onInputChange({
              ...taskDesc,
              pickup: {
                ...taskDesc.pickup,
                payload: {
                  ...taskDesc.pickup.payload,
                  quantity: val,
                },
              },
            });
          }}
        />
      </Grid>
      <Grid item xs={6}>
        <Autocomplete
          id="dropoff-location"
          freeSolo
          fullWidth
          options={Object.keys(dropoffPoints)}
          value={taskDesc.dropoff.place}
          onChange={(_ev, newValue) => {
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
            <TextField {...params} label="Dropoff Location" required={true} />
          )}
        />
      </Grid>
      <Grid item xs={4}>
        <TextField
          id="dropoff_sku"
          fullWidth
          label="Dropoff SKU"
          value={taskDesc.dropoff.payload.sku}
          required
          onChange={(ev) => {
            onInputChange({
              ...taskDesc,
              dropoff: {
                ...taskDesc.dropoff,
                payload: {
                  ...taskDesc.dropoff.payload,
                  sku: ev.target.value,
                },
              },
            });
          }}
        />
      </Grid>
      <Grid item xs={2}>
        <PositiveIntField
          id="dropoff_quantity"
          label="Quantity"
          value={taskDesc.dropoff.payload.quantity}
          onChange={(_ev, val) => {
            onInputChange({
              ...taskDesc,
              dropoff: {
                ...taskDesc.dropoff,
                payload: {
                  ...taskDesc.dropoff.payload,
                  quantity: val,
                },
              },
            });
          }}
        />
      </Grid>
    </Grid>
  );
}
