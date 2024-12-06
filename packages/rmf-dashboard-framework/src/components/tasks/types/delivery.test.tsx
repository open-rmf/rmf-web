import { fireEvent, render, screen, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { describe, expect, it, vi } from 'vitest';

import {
  DeliveryTaskDefinition,
  DeliveryTaskForm,
  isDeliveryTaskDescriptionValid,
  makeDefaultDeliveryTaskDescription,
  makeDeliveryTaskBookingLabel,
  makeDeliveryTaskShortDescription,
} from './delivery';

const mockPickupPoints = {
  pickup_1: 'handler_1',
  pickup_2: 'handler_2',
};
const mockDropoffPoints = {
  dropoff_1: 'handler_3',
  dropoff_2: 'handler_4',
};

describe('Delivery task form', () => {
  it('Delivery task form renders, changes and validates', async () => {
    const onChange = vi.fn();
    const onValidate = vi.fn();

    render(
      <DeliveryTaskForm
        taskDesc={makeDefaultDeliveryTaskDescription()}
        pickupPoints={mockPickupPoints}
        dropoffPoints={mockDropoffPoints}
        onChange={onChange}
        onValidate={onValidate}
      />,
    );

    let triggerCount = 1;

    const pickupPlace = screen.getByTestId('pickup-location');
    let input = within(pickupPlace).getByLabelText(/pickup location/i);
    pickupPlace.focus();
    fireEvent.change(input, { target: { value: 'a' } });
    fireEvent.keyDown(pickupPlace, { key: 'ArrowDown' });
    fireEvent.keyDown(pickupPlace, { key: 'Enter' });
    expect(onChange).toHaveBeenCalledTimes(triggerCount);
    expect(onValidate).toHaveBeenCalledTimes(triggerCount);
    triggerCount += 1;

    const pickupPayload = screen.getByTestId('pickup-sku');
    pickupPayload.focus();
    input = within(pickupPayload).getByLabelText(/pickup sku/i);
    fireEvent.change(input, { target: { value: 'coke' } });
    expect(onChange).toHaveBeenCalledTimes(triggerCount);
    expect(onValidate).toHaveBeenCalledTimes(triggerCount);
    triggerCount += 1;

    const pickupQuantity = screen.getByTestId('pickup-quantity');
    pickupQuantity.focus();
    input = within(pickupQuantity).getByLabelText(/quantity/i);
    await userEvent.type(input, '1');
    expect(onChange).toHaveBeenCalledTimes(triggerCount);
    expect(onValidate).toHaveBeenCalledTimes(triggerCount);
    triggerCount += 1;

    const dropoffPlace = screen.getByTestId('dropoff-location');
    dropoffPlace.focus();
    input = within(dropoffPlace).getByLabelText(/dropoff location/i);
    fireEvent.change(input, { target: { value: 'a' } });
    fireEvent.keyDown(dropoffPlace, { key: 'ArrowDown' });
    fireEvent.keyDown(dropoffPlace, { key: 'Enter' });
    expect(onChange).toHaveBeenCalledTimes(triggerCount);
    expect(onValidate).toHaveBeenCalledTimes(triggerCount);
    triggerCount += 1;

    const dropoffPayload = screen.getByTestId('dropoff-sku');
    dropoffPayload.focus();
    input = within(dropoffPayload).getByLabelText(/dropoff sku/i);
    fireEvent.change(input, { target: { value: 'coke' } });
    expect(onChange).toHaveBeenCalledTimes(triggerCount);
    expect(onValidate).toHaveBeenCalledTimes(triggerCount);
    triggerCount += 1;

    const dropoffQuantity = screen.getByTestId('dropoff-quantity');
    pickupQuantity.focus();
    input = within(dropoffQuantity).getByLabelText(/quantity/i);
    await userEvent.type(input, '1');
    expect(onChange).toHaveBeenCalledTimes(triggerCount);
    expect(onValidate).toHaveBeenCalledTimes(triggerCount);
    triggerCount += 1;
  });

  it('booking label', () => {
    const desc = makeDefaultDeliveryTaskDescription();
    desc.pickup = {
      handler: 'handler_1',
      place: 'pickup_1',
      payload: {
        sku: 'coke',
        quantity: 1,
      },
    };
    desc.dropoff = {
      handler: 'handler_3',
      place: 'dropoff_1',
      payload: {
        sku: 'coke',
        quantity: 1,
      },
    };
    const label = makeDeliveryTaskBookingLabel(desc);
    expect(label.task_definition_id).toBe(DeliveryTaskDefinition.taskDefinitionId);
    expect(label.pickup).toBe('pickup_1');
    expect(label.destination).toBe('dropoff_1');
    expect(label.payload).toBe('coke');
  });

  it('validity', () => {
    const desc = makeDefaultDeliveryTaskDescription();
    expect(isDeliveryTaskDescriptionValid(desc)).not.toBeTruthy();

    desc.pickup = {
      handler: 'handler_1',
      place: 'pickup_1',
      payload: {
        sku: 'coke',
        quantity: 1,
      },
    };
    desc.dropoff = {
      handler: 'handler_3',
      place: 'dropoff_1',
      payload: {
        sku: 'coke',
        quantity: 1,
      },
    };
    expect(isDeliveryTaskDescriptionValid(desc)).toBeTruthy();
  });

  it('short description', () => {
    const desc = makeDefaultDeliveryTaskDescription();
    desc.pickup = {
      handler: 'handler_1',
      place: 'pickup_1',
      payload: {
        sku: 'coke',
        quantity: 1,
      },
    };
    desc.dropoff = {
      handler: 'handler_3',
      place: 'dropoff_1',
      payload: {
        sku: 'coke',
        quantity: 1,
      },
    };
    expect(makeDeliveryTaskShortDescription(desc, undefined)).toBe(
      '[Delivery] Pickup [coke] from [pickup_1], dropoff [coke] at [dropoff_1]',
    );
  });
});
