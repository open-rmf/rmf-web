import React from 'react';
import { RobotSummaryState } from '../../lib';
import { render } from '@testing-library/react';
import { robotSummary } from './test.utils';

test('smoke test', () => {
  render(<RobotSummaryState item={'robot'} itemSummary={robotSummary} onClick={jest.fn()} />);
});
