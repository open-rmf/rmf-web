import React from 'react';
import { SystemSummaryItemState } from '../../lib';
import { itemSummary } from './test.utils';
import { render } from '@testing-library/react';

test('smoke test', () => {
  render(<SystemSummaryItemState itemSummary={itemSummary} onClick={jest.fn()} />);
});

test('idle and charging panels to be rendered when item value is Robots', () => {
  const itemSummaryError = { ...itemSummary, item: 'Robots' };
  const root = render(
    <SystemSummaryItemState itemSummary={itemSummaryError} onClick={jest.fn()} />,
  );
  expect(root.queryByText('Idle')).toBeTruthy();
  expect(root.queryByText('Charging')).toBeTruthy();
});
