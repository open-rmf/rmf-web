import { render } from '@testing-library/react';
import React from 'react';
import { DispenserState as RmfDispenserState } from 'rmf-models';
import { makeDispenser, makeDispenserState } from './test-utils.spec';
import { WorkcellTable } from './workcell-table';

describe('Workcell table', () => {
  it('should render properly', () => {
    const workcells = [
      makeDispenser(),
      makeDispenser({ guid: 'test1' }),
      makeDispenser({ guid: 'test2' }),
      makeDispenser({ guid: 'test3' }),
    ];
    const workcellStates = {
      test: makeDispenserState(),
      test1: makeDispenserState({ mode: RmfDispenserState.BUSY }),
      test2: makeDispenserState({ mode: RmfDispenserState.OFFLINE }),
      test3: makeDispenserState({ mode: -1 }),
    };
    const root = render(<WorkcellTable workcells={workcells} workcellStates={workcellStates} />);

    // check if all dispensers are rendered
    expect(root.getByLabelText('test')).toBeTruthy();
    expect(root.getByLabelText('test1')).toBeTruthy();
    expect(root.getByLabelText('test2')).toBeTruthy();
    expect(root.getByLabelText('test3')).toBeTruthy();

    // check if state unknown dispenser state is handled
    expect(root.getAllByText('N/A').length).toEqual(1);
  });
});
