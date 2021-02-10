import React from 'react';
import { SystemSummarySpoiltItems } from '../../lib';
import { render, screen } from '@testing-library/react';

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
