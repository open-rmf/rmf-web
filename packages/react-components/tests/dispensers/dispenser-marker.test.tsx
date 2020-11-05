import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DispenserMarker } from '../../lib';

test('triggers onClick callback when button is clicked', () => {
  const handler = jest.fn();
  const root = render(
    <svg>
      <DispenserMarker guid="test" onClick={handler} data-testid="marker" />
    </svg>,
  );
  userEvent.click(root.getByTestId('marker'));
  expect(handler).toBeCalled();
});

test('smoke test - marker with image icon', () => {
  render(
    <svg>
      <DispenserMarker guid="test" iconPath="/resources/ros-health.png" />
    </svg>,
  );
});
