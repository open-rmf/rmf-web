import { Meta, Story } from '@storybook/react';
import React from 'react';

import { WorkcellPanel } from './workcell-panel';
import { makeDispenser, makeDispenserState } from './test-utils.spec';

export default {
  title: 'Workcell Panel',
  component: WorkcellPanel,
} as Meta;

const dispensers = [makeDispenser({ guid: 'test_dispenser' })];
const ingestors = [makeDispenser({ guid: 'test_ingestor' })];

export const WorkcellPanelStory: Story = (args) => {
  return (
    <WorkcellPanel
      dispensers={dispensers}
      ingestors={ingestors}
      workCellStates={{
        test_dispenser: makeDispenserState({ guid: 'test_dispenser' }),
        test_ingestor: makeDispenserState({ guid: 'test_ingestor' }),
      }}
      workcellContext={{}}
      {...args}
    />
  );
};
