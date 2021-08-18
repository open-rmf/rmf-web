import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';

import { DoorPanel } from './door-panel';
import { doorStates, makeDetailedDoors } from './test-utils.spec';

function renderDoorPanel() {
  const mockControlClickSubmit = jasmine.createSpy();
  return render(
    <DoorPanel
      doors={makeDetailedDoors()}
      doorStates={doorStates}
      onDoorControlClick={mockControlClickSubmit}
    />,
  );
}

describe('Door Panel', () => {
  let root: ReturnType<typeof renderDoorPanel>;
  let mockControlClickSubmit: jasmine.Spy<jasmine.Func>;

  beforeEach(() => {
    mockControlClickSubmit = jasmine.createSpy();
    root = render(
      <DoorPanel
        doors={makeDetailedDoors()}
        doorStates={doorStates}
        onDoorControlClick={mockControlClickSubmit}
      />,
    );
  });

  it('should call onDoorControlClick when Open/Close button is clicked', () => {
    userEvent.click(root.getByLabelText('hardware_door_open'));
    expect(mockControlClickSubmit).toHaveBeenCalled();
  });

  it('layout view should change when view mode button is clicked', () => {
    userEvent.click(root.getByLabelText('view-mode'));
    expect(root.getByLabelText('door-table')).toBeTruthy();
  });
});
