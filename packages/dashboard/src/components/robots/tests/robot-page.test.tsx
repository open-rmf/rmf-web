import { render } from '@testing-library/react';
import React from 'react';
import { RobotPage } from '../robot-page';

describe('RobotPage', () => {
  it('smoke test', () => {
    render(<RobotPage />);
  });
});
