import { render } from '@testing-library/react';
import React from 'react';
import { TaskPage } from '../task-page';

describe('TaskPage', () => {
  it('smoke test', () => {
    render(<TaskPage />);
  });
});
