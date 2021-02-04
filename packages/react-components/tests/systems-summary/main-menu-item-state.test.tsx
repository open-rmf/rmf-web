import React from 'react';
import { MainMenuItemState } from '../../lib';
import { itemSummary } from './test.utils';
import { render } from '@testing-library/react';

test('smoke test', () => {
  render(<MainMenuItemState itemSummary={itemSummary} onClick={jest.fn()} />);
});
