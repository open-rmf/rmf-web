import { cleanup, render, fireEvent } from '@testing-library/react';
import React from 'react';
import { allLiftModes, allLiftMotion, makeLiftState } from '../lifts/test-utils.spec';
import { LiftMarker, LiftMarkerProps } from './lift-marker';

describe('LiftMarker', () => {
  it('smoke test with different variants', () => {
    (
      [
        'emergency',
        'fire',
        'human',
        'moving',
        'offLine',
        'onCurrentFloor',
        'unknown',
        undefined,
      ] as LiftMarkerProps['variant'][]
    ).forEach((variant) => {
      render(
        <svg>
          <LiftMarker
            cx={0}
            cy={0}
            width={1}
            height={1}
            yaw={0}
            liftState={makeLiftState()}
            variant={variant}
          />
        </svg>,
      );
      cleanup();
    });
  });

  it('smoke test with translate false', () => {
    render(
      <svg>
        <LiftMarker cx={0} cy={0} width={1} height={1} yaw={0} liftState={makeLiftState()} />
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
            <LiftMarker cx={0} cy={0} width={1} height={1} yaw={0} liftState={state} />
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
            <LiftMarker cx={0} cy={0} width={1} height={1} yaw={0} liftState={state} />
          </svg>,
        );
        cleanup();
      });
  });

  it('smoke test with no state', () => {
    render(
      <svg>
        <LiftMarker cx={0} cy={0} width={1} height={1} yaw={0} />
      </svg>,
    );
  });

  it('trigger onClick event', () => {
    const mockOnClick = jasmine.createSpy();
    const root = render(
      <svg>
        <LiftMarker
          cx={0}
          cy={0}
          width={1}
          height={1}
          yaw={0}
          liftState={makeLiftState()}
          onClick={mockOnClick}
          data-testid="marker"
        />
      </svg>,
    );
    fireEvent.click(root.getByTestId('marker'));
    expect(mockOnClick).toHaveBeenCalled();
  });
});
