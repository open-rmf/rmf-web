import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DoorMarker } from '../../lib';
import { makeDoor } from './test-utils';

test('triggers onClick callback when button is clicked', () => {
  const handler = jest.fn();
  const root = render(
    <svg>
      <DoorMarker door={makeDoor()} onClick={handler} data-testid="marker" />
    </svg>,
  );
  userEvent.click(root.getByTestId('marker'));
  expect(handler).toBeCalled();
});
