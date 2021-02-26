import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { cleanup, render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DoorMarker } from '../../lib';
import { makeDoor } from './test-utils';

it('smoke test with different door modes', () => {
  ([
    { value: RomiCore.DoorMode.MODE_CLOSED },
    { value: RomiCore.DoorMode.MODE_MOVING },
    { value: RomiCore.DoorMode.MODE_OPEN },
    { value: -1 },
  ] as RomiCore.DoorMode[]).forEach((mode) => {
    render(
      <svg>
        <DoorMarker door={makeDoor()} doorMode={mode} />
      </svg>,
    );
    cleanup();
  });
});

it('smoke test with different door types', () => {
  [
    makeDoor({
      door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SLIDING,
    }),
    makeDoor({
      door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SWING,
    }),
    makeDoor({
      door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_TELESCOPE,
    }),
    makeDoor({
      door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING,
    }),
    makeDoor({
      door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SWING,
    }),
    makeDoor({
      door_type: RomiCore.Door.DOOR_TYPE_SINGLE_TELESCOPE,
    }),
    makeDoor({
      door_type: RomiCore.Door.DOOR_TYPE_UNDEFINED,
    }),
    makeDoor({
      door_type: -1,
    }),
  ].forEach((door) => {
    render(
      <svg>
        <DoorMarker door={door} />
      </svg>,
    );
    cleanup();
  });
});

it('triggers onClick callback when button is clicked', () => {
  const mockOnClick = jasmine.createSpy();

  const root = render(
    <svg>
      <DoorMarker door={makeDoor()} onClick={mockOnClick} data-testid="marker" />
    </svg>,
  );
  userEvent.click(root.getByTestId('marker'));
  expect(mockOnClick).toHaveBeenCalled();
});
