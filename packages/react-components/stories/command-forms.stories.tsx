import { Meta, Story } from '@storybook/react';
import React from 'react';
import { LoopRequestForm } from '../lib';

export default {
  title: 'Command Forms',
  argTypes: { doLoopRequest: { action: 'loop request' } },
} as Meta;

const fleets = ['fleetA', 'fleetB'];

const availablePlaces = {
  fleetA: ['placeA', 'placeB'],
  fleetB: ['placeB', 'placeC'],
};

export const LoopRequest: Story = (args) => (
  <LoopRequestForm fleetNames={fleets} availablePlaces={availablePlaces} {...args} />
);
