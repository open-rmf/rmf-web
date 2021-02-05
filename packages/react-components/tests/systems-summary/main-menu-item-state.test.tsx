import React from 'react';
import { MainMenuItemState } from '../../lib';
import { itemSummary } from './test.utils';
import { render } from '@testing-library/react';

test('smoke test', () => {
  render(<MainMenuItemState itemSummary={itemSummary} onClick={jest.fn()} />);
});

test('idle and charging panels to be rendered when item value is Robots', () => {
  const itemSummaryError = { ...itemSummary, item: 'Robots' };
  const root = render(<MainMenuItemState itemSummary={itemSummaryError} onClick={jest.fn()} />);
  expect(root.queryByText('Idle')).toBeTruthy();
  expect(root.queryByText('Charging')).toBeTruthy();
});
