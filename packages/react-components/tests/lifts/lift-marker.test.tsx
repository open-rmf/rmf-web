import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { LiftMarker, LiftMarkerProps } from '../../lib';
import { makeLift, makeLiftState } from './test-utils';

([
  'emergency',
  'fire',
  'human',
  'moving',
  'offLine',
  'onCurrentFloor',
  'unknown',
] as LiftMarkerProps['variant'][]).forEach((variant) => {
  test(`smoke test - ${variant}`, () => {
    render(
      <svg>
        <LiftMarker lift={makeLift()} liftState={makeLiftState()} variant={variant} />
      </svg>,
    );
  });
});

test('trigger onClick event', () => {
  const handler = jest.fn();
  const root = render(
    <svg>
      <LiftMarker
        lift={makeLift()}
        liftState={makeLiftState()}
        onClick={handler}
        data-testid="marker"
      />
    </svg>,
  );
  userEvent.click(root.getByTestId('marker'));
  expect(handler).toHaveBeenCalled();
});
