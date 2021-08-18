import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';

import { DispenserPanel } from './dispenser-panel';
import { makeDispenser, makeDispenserState } from './test-utils.spec';

function renderDispenserPanel() {
  return render(
    <DispenserPanel
      dispensers={[makeDispenser()]}
      dispenserStates={{ test: makeDispenserState() }}
    />,
  );
}

describe('Dispenser Panel', () => {
  let root: ReturnType<typeof renderDispenserPanel>;

  beforeEach(() => {
    root = render(
      <DispenserPanel
        dispensers={[makeDispenser()]}
        dispenserStates={{ test: makeDispenserState() }}
      />,
    );
  });

  it('layout view should change when view mode button is clicked', () => {
    userEvent.click(root.getByLabelText('view-mode'));
    expect(root.getByLabelText('dispenser-table')).toBeTruthy();
  });
});
