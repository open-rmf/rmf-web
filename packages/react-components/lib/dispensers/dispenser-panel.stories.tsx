import { Meta, Story } from '@storybook/react';
import React from 'react';

import { DispenserPanel } from './dispenser-panel';
import { makeDispenser, makeDispenserState } from './test-utils.spec';

export default {
  title: 'Dispenser Panel',
  component: DispenserPanel,
} as Meta;

export const DispenserPanelStory: Story = (args) => {
  return (
    <DispenserPanel
      dispensers={[makeDispenser()]}
      dispenserStates={{ test: makeDispenserState() }}
      {...args}
    />
  );
};
