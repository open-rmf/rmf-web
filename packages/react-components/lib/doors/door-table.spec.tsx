import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DoorTable } from './door-table';
import { doorStates, makeDoorsData } from './test-utils.spec';

describe('door table', () => {
  it('should render properly', () => {
    const root = render(<DoorTable doors={makeDoorsData()} doorStates={doorStates} />);

    // test to see if door names are rendered
    expect(root.getByText('main_door')).toBeTruthy();
    expect(root.getByText('hardware_door')).toBeTruthy();
    expect(root.getByText('coe_door')).toBeTruthy();
    expect(root.getByText('exit_door')).toBeTruthy();
    expect(root.getByText('extra_door')).toBeTruthy();

    // test if op mode is displayed correctly
    expect(root.getAllByText('Online').length).toEqual(3);
    expect(root.getAllByText('Offline').length).toEqual(2);

    // test if door state is displayed correctly
    expect(root.getAllByText('N/A').length).toEqual(2);
    expect(root.getAllByText('CLOSED').length).toEqual(1);
    expect(root.getAllByText('OPEN').length).toEqual(1);
    expect(root.getAllByText('MOVING').length).toEqual(1);
  });

  it('should call onDoorControlClick when OPEN button is clicked', () => {
    const onControlClickSpy = jasmine.createSpy();
    const root = render(
      <DoorTable
        doors={makeDoorsData()}
        doorStates={doorStates}
        onDoorControlClick={onControlClickSpy}
      />,
    );

    userEvent.click(root.getByLabelText('hardware_door_open'));
    expect(onControlClickSpy).toHaveBeenCalled();
  });

  it('should call onDoorControlClick when CLOSE button is clicked', () => {
    const onControlClickSpy = jasmine.createSpy();
    const root = render(
      <DoorTable
        doors={makeDoorsData()}
        doorStates={doorStates}
        onDoorControlClick={onControlClickSpy}
      />,
    );

    userEvent.click(root.getByLabelText('hardware_door_close'));
    expect(onControlClickSpy).toHaveBeenCalled();
  });
});
