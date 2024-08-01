import { fireEvent, render } from '@testing-library/react';

import { makeDispenser, makeDispenserState } from './test-utils.spec';
import { WorkcellPanel } from './workcell-panel';

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
      />,
    );
  });

  it('layout view should change when view mode button is clicked', () => {
    fireEvent.click(root.getByLabelText('view mode'));
    expect(root.getAllByLabelText('workcell-table')).toBeTruthy();
  });
});
