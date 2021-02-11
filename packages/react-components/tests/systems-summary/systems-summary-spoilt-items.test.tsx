import React from 'react';
import { SystemSummarySpoiltItems } from '../../lib';
import { render, screen, fireEvent } from '@testing-library/react';
import { door, lift, fleet } from './test.utils';

test('should render the list of spoilt items', () => {
  const spoiltItems = [
    { type: 'door', name: 'door', itemNameAndState: 'item - state' },
    { type: 'robot', name: 'robot', itemNameAndState: 'item - state', errorMessage: 'error' },
  ];
  let errorMessageCount = 0;
  spoiltItems.forEach((item) => {
    if (item.errorMessage !== undefined) errorMessageCount += 1;
  });
  render(
    <SystemSummarySpoiltItems
      doors={[]}
      lifts={[]}
      dispensers={[]}
      robots={{}}
      spoiltItems={spoiltItems}
    />,
  );

  expect(screen.getAllByText('item - state').length).toEqual(spoiltItems.length);
  expect(screen.getAllByText('Error - error').length).toEqual(errorMessageCount);
});

test('it should trigger the spoilt item callback function when spoit item is clicked', () => {
  const spoiltItems = [
    { type: 'door', name: 'door', itemNameAndState: 'door - state' },
    { type: 'lift', name: 'lift', itemNameAndState: 'lift - state' },
    { type: 'dispenser', name: 'dispenser', itemNameAndState: 'dispenser - state' },
    {
      type: 'robot',
      name: 'robot',
      fleet: 'fleet',
      itemNameAndState: 'robot - state',
      errorMessage: 'error',
    },
  ];
  const doorClick = jest.fn();
  const liftClick = jest.fn();
  const dispenserClick = jest.fn();
  const robotClick = jest.fn();

  const { debug } = render(
    <SystemSummarySpoiltItems
      doors={[door]}
      lifts={[lift]}
      dispensers={['dispenser']}
      robots={{ fleet: fleet }}
      spoiltItems={spoiltItems}
      spoiltDoorClick={doorClick}
      spoiltLiftClick={liftClick}
      spoiltDispenserClick={dispenserClick}
      spoiltRobotClick={robotClick}
    />,
  );
  debug();
  fireEvent.click(screen.getByText('door - state'));
  fireEvent.click(screen.getByText('lift - state'));
  fireEvent.click(screen.getByText('dispenser - state'));
  fireEvent.click(screen.getByText('robot - state'));

  expect(doorClick).toBeCalled();
  expect(liftClick).toBeCalled();
  expect(dispenserClick).toBeCalled();
  expect(robotClick).toBeCalled();
});
