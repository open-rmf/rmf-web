import { render } from '@testing-library/react';
import React from 'react';
import { LiftAccordion } from '..';
import { makeLift } from './test-utils';

test(`smoke test`, () => {
  render(<LiftAccordion lift={makeLift()} />);
});
