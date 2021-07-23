import { Meta, Story } from '@storybook/react';
import React from 'react';
import { DeliveryRequestForm } from './delivery-request-form';
import { LoopRequestForm } from './loop-request-form';
import { availableDispensers, availablePlaces, fleets } from './test-data.spec';

export default {
  title: 'Command Forms',
  argTypes: {
    doLoopRequest: { action: 'loop request' },
    doDeliveryRequest: { action: 'delivery request' },
  },
} as Meta;

export const DeliveryRequest: Story = (args) => (
  // add a background to visualize different theme colors properly
  <div style={{ backgroundColor: '#A8A8A8', padding: '1rem' }}>
    <DeliveryRequestForm
      fleetNames={fleets}
      availablePlaces={availablePlaces}
      availableDispensers={availableDispensers}
      {...args}
    />
  </div>
);

export const LoopRequest: Story = (args) => (
  // add a background to visualize different theme colors properly
  <div style={{ backgroundColor: '#A8A8A8', padding: '1rem' }}>
    <LoopRequestForm fleetNames={fleets} availablePlaces={availablePlaces} {...args} />
  </div>
);
