import { cleanup, render, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';

import { DoorPanel } from './door-panel';
import { doorStates, doors } from './test-utils.spec';
import { DetailedDoor } from './utils';

const makeDetailedDoors = (): DetailedDoor[] => {
  return doors.map((door) => ({ ...door, level: 'L1' }));
};

describe('Door Panel', () => {
  it('smoke test with different door types and states', () => {
    render(<DoorPanel doors={makeDetailedDoors()} doorStates={doorStates} />);
  });

  it('should call onDoorControlClick when Open/Close button is clicked', () => {
    const mockDoorControl = jasmine.createSpy();
    const panel = render(
      <DoorPanel
        doors={makeDetailedDoors()}
        doorStates={doorStates}
        onDoorControlClick={mockDoorControl}
      />,
    );
    userEvent.click(panel.getByLabelText('hardware_door_open'));
    expect(mockDoorControl).toHaveBeenCalled();
  });

  it('should call onDoorControlClick when Open/Close button is clicked', () => {
    const panel = render(<DoorPanel doors={makeDetailedDoors()} doorStates={doorStates} />);
    userEvent.click(panel.getByLabelText('view-mode'));
    expect(panel.getByLabelText('door-table')).toBeTruthy();
  });
});
