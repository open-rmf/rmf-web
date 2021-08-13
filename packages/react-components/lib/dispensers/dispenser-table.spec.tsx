import { render } from '@testing-library/react';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { DispenserTable } from './dispenser-table';
import { makeDispenser, makeDispenserState } from './test-utils.spec';

describe('Dispenser table', () => {
  it('should render properly', () => {
    const dispensers = [
      makeDispenser(),
      makeDispenser({ guid: 'test1' }),
      makeDispenser({ guid: 'test2' }),
      makeDispenser({ guid: 'test3' }),
    ];
    const dispenserStates = {
      test: makeDispenserState(),
      test1: makeDispenserState({ mode: RmfModels.DispenserState.BUSY }),
      test2: makeDispenserState({ mode: RmfModels.DispenserState.OFFLINE }),
      test3: makeDispenserState({ mode: -1 }),
    };
    render(<DispenserTable dispensers={dispensers} dispenserStates={dispenserStates} />);
  });
});
