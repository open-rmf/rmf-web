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
    const root = render(
      <DispenserTable dispensers={dispensers} dispenserStates={dispenserStates} />,
    );

    // check if all dispensers are rendered
    expect(root.getByLabelText('test')).toBeTruthy();
    expect(root.getByLabelText('test1')).toBeTruthy();
    expect(root.getByLabelText('test2')).toBeTruthy();
    expect(root.getByLabelText('test3')).toBeTruthy();

    // check if state unknown dispenser state is handled
    expect(root.getAllByText('N/A').length).toEqual(1);
  });
});
