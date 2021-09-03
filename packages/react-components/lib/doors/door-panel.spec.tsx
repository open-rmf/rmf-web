import { render, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DoorPanel } from './door-panel';
import { doorStates, makeDoorsData } from './test-utils.spec';

function renderDoorPanel() {
  return render(<DoorPanel doors={makeDoorsData()} doorStates={doorStates} />);
}

describe('Door Panel', () => {
  let root: ReturnType<typeof renderDoorPanel>;
  let mockControlClickSubmit: jasmine.Spy<jasmine.Func>;

  beforeEach(() => {
    mockControlClickSubmit = jasmine.createSpy();
    root = render(
      <DoorPanel
        doors={makeDoorsData()}
        doorStates={doorStates}
        onDoorControlClick={mockControlClickSubmit}
      />,
    );
  });

  it('should call onDoorControlClick when Open/Close button is clicked', () => {
    const doorCell = root.getByRole('region', { name: 'main_door' });
    const openBtn = within(doorCell).getByRole('button', { name: 'Open' });
    openBtn.click();
    expect(mockControlClickSubmit).toHaveBeenCalled();
  });

  it('layout view should change when view mode button is clicked', () => {
    userEvent.click(root.getByLabelText('view mode'));
    expect(root.getByLabelText('door-table')).toBeTruthy();
  });
});
