import { Meta, Story } from '@storybook/react';
import React from 'react';

import { DoorPanel } from './door-panel';
import { doors, doorStates } from './test-utils.spec';
import { DoorData } from './utils';

export default {
  title: 'Door Panel',
  component: DoorPanel,
} as Meta;

const detailedDoors: DoorData[] = doors.map((door, i) => ({ door, level: 'L1', id: i }));

export const DoorSidePanel: Story = (args) => {
  return <DoorPanel doors={detailedDoors} doorStates={doorStates} {...args} />;
};
