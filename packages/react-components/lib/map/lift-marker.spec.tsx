import { cleanup, render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { allLiftModes, allLiftMotion, makeLift, makeLiftState } from '../lifts/test-utils.spec';
import { LiftMarker, LiftMarkerProps } from './lift-marker';

describe('LiftMarker', () => {
  it('smoke test with different variants', () => {
    ([
      'emergency',
      'fire',
      'human',
      'moving',
      'offLine',
      'onCurrentFloor',
      'unknown',
      undefined,
    ] as LiftMarkerProps['variant'][]).forEach((variant) => {
      render(
        <svg>
          <LiftMarker lift={makeLift()} liftState={makeLiftState()} variant={variant} />
        </svg>,
      );
      cleanup();
    });
  });

  it('smoke test with translate false', () => {
    render(
      <svg>
        <LiftMarker lift={makeLift()} liftState={makeLiftState()} />
      </svg>,
    );
  });

  it('smoke test with different modes', () => {
    allLiftModes()
      .map((mode) =>
        makeLiftState({
          current_mode: mode,
        }),
      )
      .forEach((state) => {
        render(
          <svg>
            <LiftMarker lift={makeLift()} liftState={state} />
          </svg>,
        );
        cleanup();
      });
  });

  it('smoke test with different motions', () => {
    allLiftMotion()
      .map((motion) =>
        makeLiftState({
          motion_state: motion,
        }),
      )
      .forEach((state) => {
        render(
          <svg>
            <LiftMarker lift={makeLift()} liftState={state} />
          </svg>,
        );
        cleanup();
      });
  });

  it('smoke test with no state', () => {
    render(
      <svg>
        <LiftMarker lift={makeLift()} />
      </svg>,
    );
  });

  it('trigger onClick event', () => {
    const mockOnClick = jasmine.createSpy();
    const root = render(
      <svg>
        <LiftMarker
          lift={makeLift()}
          liftState={makeLiftState()}
          onClick={mockOnClick}
          data-testid="marker"
        />
      </svg>,
    );
    userEvent.click(root.getByTestId('marker'));
    expect(mockOnClick).toHaveBeenCalled();
  });
});
