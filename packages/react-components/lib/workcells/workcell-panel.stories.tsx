import { Meta, StoryFn } from '@storybook/react';
import React from 'react';

import { WorkcellPanel } from './workcell-panel';
import { makeDispenser, makeDispenserState } from './test-utils.spec';

export default {
  title: 'Workcell Panel',
  component: WorkcellPanel,
} satisfies Meta;

const dispensers = [
  makeDispenser({ guid: 'test_dispenser' }),
  makeDispenser({ guid: 'test_dispenser1' }),
  makeDispenser({ guid: 'test_dispenser2' }),
  makeDispenser({ guid: 'test_dispenser3' }),
  makeDispenser({ guid: 'test_dispenser4' }),
  makeDispenser({ guid: 'test_dispenser5' }),
  makeDispenser({ guid: 'test_dispenser6' }),
];
const ingestors = [
  makeDispenser({ guid: 'test_ingestor' }),
  makeDispenser({ guid: 'test_ingestor1' }),
  makeDispenser({ guid: 'test_ingestor2' }),
  makeDispenser({ guid: 'test_ingestor3' }),
  makeDispenser({ guid: 'test_ingestor4' }),
  makeDispenser({ guid: 'test_ingestor5' }),
];

export const WorkcellPanelStory: StoryFn = (args) => {
  return (
    <WorkcellPanel
      dispensers={dispensers}
      ingestors={ingestors}
      workcellStates={{
        test_dispenser: makeDispenserState({ guid: 'test_dispenser' }),
        test_dispenser1: makeDispenserState({ guid: 'test_dispenser1' }),
        test_dispenser3: makeDispenserState({ guid: 'test_dispenser3' }),
        test_ingestor: makeDispenserState({ guid: 'test_ingestor' }),
        test_ingestor2: makeDispenserState({ guid: 'test_ingestor2' }),
        test_ingestor4: makeDispenserState({ guid: 'test_ingestor4' }),
      }}
      {...args}
    />
  );
};
