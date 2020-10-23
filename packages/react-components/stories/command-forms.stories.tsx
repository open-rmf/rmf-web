import { Meta, Story } from '@storybook/react';
import React from 'react';
import { DeliveryRequestForm, LoopRequestForm } from '../lib';
import { availableDispensers, availablePlaces, fleets } from '../lib/commands/tests/test-data';

export default {
  title: 'Command Forms',
  argTypes: {
    doLoopRequest: { action: 'loop request' },
    doDeliveryRequest: { action: 'delivery request' },
  },
} as Meta;

export const DeliveryRequest: Story = (args) => (
  <DeliveryRequestForm
    fleetNames={fleets}
    availablePlaces={availablePlaces}
    availableDispensers={availableDispensers}
    {...args}
  />
);

export const LoopRequest: Story = (args) => (
  <LoopRequestForm fleetNames={fleets} availablePlaces={availablePlaces} {...args} />
);
