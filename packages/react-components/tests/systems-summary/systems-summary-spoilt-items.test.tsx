import React from 'react';
import { SystemSummarySpoiltItems } from '../../lib';
import { render, screen } from '@testing-library/react';

test('should render the list of spoilt items', () => {
  const spoiltItems = [
    { itemNameAndState: 'item - state' },
    { itemNameAndState: 'item - state', errorMessage: 'error' },
  ];
  let errorMessageCount = 0;
  spoiltItems.forEach((item) => {
    if (item.errorMessage !== undefined) errorMessageCount += 1;
  });
  render(<SystemSummarySpoiltItems spoiltItems={spoiltItems} />);

  expect(screen.getAllByText('item - state').length).toEqual(spoiltItems.length);
  expect(screen.getAllByText('Error - error').length).toEqual(errorMessageCount);
});
