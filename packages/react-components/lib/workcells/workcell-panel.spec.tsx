import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';

import { WorkcellPanel } from './workcell-panel';
import { makeDispenser, makeDispenserState } from './test-utils.spec';

function renderWorkcellPanel() {
  const dispensers = [makeDispenser({ guid: 'test_dispenser' })];
  const ingestors = [makeDispenser({ guid: 'test_ingestor' })];
  return render(
    <WorkcellPanel
      dispensers={dispensers}
      ingestors={ingestors}
      workcellStates={{
        test_dispenser: makeDispenserState({ guid: 'test_dispenser' }),
        test_ingestor: makeDispenserState({ guid: 'test_ingestor' }),
      }}
      workcellContext={{}}
    />,
  );
}

describe('Workcell Panel', () => {
  let root: ReturnType<typeof renderWorkcellPanel>;

  beforeEach(() => {
    const dispensers = [makeDispenser({ guid: 'test_dispenser' })];
    const ingestors = [makeDispenser({ guid: 'test_ingestor' })];
    root = render(
      <WorkcellPanel
        dispensers={dispensers}
        ingestors={ingestors}
        workcellStates={{
          test_dispenser: makeDispenserState({ guid: 'test_dispenser' }),
          test_ingestor: makeDispenserState({ guid: 'test_ingestor' }),
        }}
        workcellContext={{}}
      />,
    );
  });

  it('layout view should change when view mode button is clicked', () => {
    userEvent.click(root.getByLabelText('view mode'));
    expect(root.getAllByLabelText('workcell-table')).toBeTruthy();
  });
});
