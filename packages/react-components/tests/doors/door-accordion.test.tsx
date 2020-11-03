import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DoorAccordion } from '../../lib';
import { makeDoor } from './test-utils';

test('triggers door control dispatch when open door button is clicked', () => {
  const handler = jest.fn();
  const root = render(<DoorAccordion door={makeDoor()} onDoorControlClick={handler} />);
  userEvent.click(root.getByText('Open'));
  expect(handler).toHaveBeenCalledWith(
    expect.anything(),
    expect.anything(),
    RomiCore.DoorMode.MODE_OPEN,
  );
});

test('triggers door control dispatch when close door button is clicked', () => {
  const handler = jest.fn();
  const root = render(<DoorAccordion door={makeDoor()} onDoorControlClick={handler} />);
  userEvent.click(root.getByText('Close'));
  expect(handler).toHaveBeenCalledWith(
    expect.anything(),
    expect.anything(),
    RomiCore.DoorMode.MODE_CLOSED,
  );
});
