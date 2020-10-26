import { render } from '@testing-library/react';
import React from 'react';
import { LiftAccordion } from '../../lib';
import { makeLift } from './test-utils';

test(`smoke test`, () => {
  render(<LiftAccordion lift={makeLift()} />);
});
