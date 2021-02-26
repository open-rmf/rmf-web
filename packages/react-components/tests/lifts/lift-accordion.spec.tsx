import { cleanup, render } from '@testing-library/react';
import React from 'react';
import { LiftAccordion } from '../../lib';
import { allLiftMotion, makeLift, makeLiftState } from './test-utils';

it('smoke test with different lift motion', () => {
  allLiftMotion()
    .map((motion) =>
      makeLiftState({
        motion_state: motion,
      }),
    )
    .forEach((state) => {
      render(<LiftAccordion lift={makeLift()} liftState={state} />);
      cleanup();
    });
});

it('smoke test without lift state', () => {
  render(<LiftAccordion lift={makeLift()} />);
});
