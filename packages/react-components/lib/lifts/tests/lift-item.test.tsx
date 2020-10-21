import { render } from '@testing-library/react';
import React from 'react';
import { LiftItem } from '..';
import { makeLift } from './test-utils';

test(`smoke test`, () => {
  render(<LiftItem lift={makeLift()} />);
});
