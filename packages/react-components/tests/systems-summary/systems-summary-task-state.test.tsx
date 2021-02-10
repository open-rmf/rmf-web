import React from 'react';
import { SystemSummaryTaskState } from '../../lib';
import { render } from '@testing-library/react';
import { tasks } from './test.utils';

test('smoke test', () => {
  const { debug } = render(<SystemSummaryTaskState tasks={tasks} onClick={jest.fn()} />);
  debug();
});
