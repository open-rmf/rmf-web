import React from 'react';
import { SystemSummaryItemState } from '../../lib';
import { itemSummary } from './test.utils';
import { render } from '@testing-library/react';

test('smoke test', () => {
  render(<SystemSummaryItemState item={'door'} itemSummary={itemSummary} onClick={jest.fn()} />);
});
