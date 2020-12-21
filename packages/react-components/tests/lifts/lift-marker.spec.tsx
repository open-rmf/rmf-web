import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { LiftMarker, LiftMarkerProps } from '../../lib';
import { mockOnClick } from '../test-utils';
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
  it(`smoke test - ${variant}`, () => {
    render(
      <svg>
        <LiftMarker lift={makeLift()} liftState={makeLiftState()} variant={variant} />
      </svg>,
    );
  });
});

it('trigger onClick event', () => {
  const handler = mockOnClick();
  spyOn(handler, 'onClick');
  const root = render(
    <svg>
      <LiftMarker
        lift={makeLift()}
        liftState={makeLiftState()}
        onClick={handler.onClick}
        data-testid="marker"
      />
    </svg>,
  );
  userEvent.click(root.getByTestId('marker'));
  expect(handler.onClick).toHaveBeenCalled();
});
