import React from 'react';
import { MainMenuTaskState } from '../../lib';
import { render } from '@testing-library/react';
import { tasks } from './test.utils';

test('smoke test', () => {
  render(<MainMenuTaskState tasks={tasks} />);
});
