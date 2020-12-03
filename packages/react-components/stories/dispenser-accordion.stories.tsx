import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { DispenserAccordion } from '../lib';

export default {
  title: 'Dispenser Accordion',
  component: DispenserAccordion,
  parameters: { actions: { argTypesRegex: '^on.*' } },
} as Meta;

const baseDispenser: RomiCore.DispenserState = {
  guid: 'test',
  mode: RomiCore.DispenserState.IDLE,
  request_guid_queue: [],
  seconds_remaining: 0,
  time: { sec: 0, nanosec: 0 },
};

export const Basic: Story = (args) => (
  <DispenserAccordion dispenser={'dispenser'} dispenserState={baseDispenser} {...args} />
);
