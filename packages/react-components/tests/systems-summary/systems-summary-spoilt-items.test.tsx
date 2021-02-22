import React from 'react';
import { SystemSummarySpoiltItems } from '../../lib';
import { render, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { door, lift, fleet } from './test.utils';

test('should render the lists of different spoilt items', () => {
  render(
    <SystemSummarySpoiltItems
      doors={[
        {
          name: 'door',
          state: 'state',
          door: door,
        },
      ]}
      lifts={[
        {
          name: 'lift',
          state: 'state',
          lift: lift,
        },
      ]}
      robots={[]}
      dispensers={[]}
    />,
  );

  expect(screen.getAllByText('door - state').length).toEqual(1);
  expect(screen.getAllByText('lift - state').length).toEqual(1);
});

test('it should trigger the spoilt item callback function when spoit item is clicked', () => {
  const doorClick = jest.fn();
  const liftClick = jest.fn();
  const dispenserClick = jest.fn();
  const robotClick = jest.fn();

  render(
    <SystemSummarySpoiltItems
      doors={[
        {
          name: 'door',
          state: 'state',
          door: door,
        },
      ]}
      lifts={[
        {
          name: 'lift',
          state: 'state',
          lift: lift,
        },
      ]}
      dispensers={[
        {
          name: 'dispenser',
          state: 'state',
          dispenser: 'dispenser',
        },
      ]}
      robots={[
        {
          name: 'robot',
          state: 'state',
          robot: fleet.robots[0],
          fleet: fleet.name,
        },
      ]}
      onClickSpoiltDoor={doorClick}
      onClickSpoiltLift={liftClick}
      onClickSpoiltDispenser={dispenserClick}
      onClickSpoiltRobot={robotClick}
    />,
  );

  userEvent.click(screen.getByText('door - state'));
  userEvent.click(screen.getByText('lift - state'));
  userEvent.click(screen.getByText('dispenser - state'));
  userEvent.click(screen.getByText('robot - state'));

  expect(doorClick).toBeCalled();
  expect(liftClick).toBeCalled();
  expect(dispenserClick).toBeCalled();
  expect(robotClick).toBeCalled();
});
