import { cleanup, render, fireEvent } from '@testing-library/react';
import React from 'react';
import { Door as RmfDoor } from 'rmf-models';
import { allDoorModes, allDoorTypes } from '../doors/test-utils.spec';
import { DoorMarker } from './door-marker';

describe('DoorMarker', () => {
  it('smoke test with different door modes', () => {
    allDoorModes().forEach((mode) => {
      render(
        <svg>
          <DoorMarker
            x1={0}
            y1={0}
            x2={1}
            y2={1}
            doorType={RmfDoor.DOOR_TYPE_SINGLE_SLIDING}
            doorMode={mode.value}
          />
        </svg>,
      );
      cleanup();
    });
  });

  it('smoke test with different door types', () => {
    allDoorTypes().forEach((type) => {
      render(
        <svg>
          <DoorMarker x1={0} y1={0} x2={1} y2={1} doorType={type} />
        </svg>,
      );
      cleanup();
    });
  });

  it('triggers onClick callback when button is clicked', () => {
    const mockOnClick = jasmine.createSpy();

    const root = render(
      <svg>
        <DoorMarker
          x1={0}
          y1={0}
          x2={1}
          y2={1}
          doorType={RmfDoor.DOOR_TYPE_SINGLE_SLIDING}
          onClick={mockOnClick}
          data-testid="marker"
        />
      </svg>,
    );
    fireEvent.click(root.getByTestId('marker'));
    expect(mockOnClick).toHaveBeenCalled();
  });
});
