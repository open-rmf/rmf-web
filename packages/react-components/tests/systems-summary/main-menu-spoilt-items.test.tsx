import React from 'react';
import { SystemSummarySpoiltItems } from '../../lib';
import { render } from '@testing-library/react';

test('smoke test', () => {
  render(<SystemSummarySpoiltItems spoiltItems={[{ itemNameAndState: 'item - state' }]} />);
});
