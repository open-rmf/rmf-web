import { Meta, StoryObj } from '@storybook/react';

import { makeDispenser, makeDispenserState } from './test-utils.spec';
import { WorkcellPanel } from './workcell-panel';

export default {
  title: 'Workcell Panel',
  component: WorkcellPanel,
} satisfies Meta;

type Story = StoryObj<typeof WorkcellPanel>;

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

export const WorkcellPanelStory: Story = {
  args: {
    dispensers,
    ingestors,
    workcellStates: {
      test_dispenser: makeDispenserState({ guid: 'test_dispenser' }),
      test_dispenser1: makeDispenserState({ guid: 'test_dispenser1' }),
      test_dispenser3: makeDispenserState({ guid: 'test_dispenser3' }),
      test_ingestor: makeDispenserState({ guid: 'test_ingestor' }),
      test_ingestor2: makeDispenserState({ guid: 'test_ingestor2' }),
      test_ingestor4: makeDispenserState({ guid: 'test_ingestor4' }),
    },
  },
};
