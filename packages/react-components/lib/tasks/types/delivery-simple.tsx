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

export interface SimpleDeliveryTaskDescription {
  pickup: TaskPlace;
  dropoff: TaskPlace;
}

function isTaskPlaceValid(place: TaskPlace): boolean {
  return (
    isNonEmptyString(place.place) &&
    isNonEmptyString(place.handler) &&
    isNonEmptyString(place.payload.sku) &&
    isPositiveNumber(place.payload.quantity)
  );
}

function isSimpleDeliveryTaskDescriptionValid(
  taskDescription: SimpleDeliveryTaskDescription,
): boolean {
  return isTaskPlaceValid(taskDescription.pickup) && isTaskPlaceValid(taskDescription.dropoff);
}

export function makeDefaultSimpleDeliveryTaskDescription(): SimpleDeliveryTaskDescription {
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

export interface SimpleDeliveryTaskFormProps {
  taskDesc: SimpleDeliveryTaskDescription;
  pickupPoints: Record<string, string>;
  dropoffPoints: Record<string, string>;
  onChange(taskDesc: SimpleDeliveryTaskDescription): void;
  allowSubmit(allow: boolean): void;
}

export function SimpleDeliveryTaskForm({
  taskDesc,
  pickupPoints = {},
  dropoffPoints = {},
  onChange,
  allowSubmit,
}: SimpleDeliveryTaskFormProps): React.JSX.Element {
  const theme = useTheme();
  const onInputChange = (desc: SimpleDeliveryTaskDescription) => {
    allowSubmit(isSimpleDeliveryTaskDescriptionValid(desc));
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
