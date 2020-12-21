import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DispenserMarker } from '../../lib';
import { mockOnClick } from '../test-utils';

it('triggers onClick callback when button is clicked', () => {
  const handler = mockOnClick();
  spyOn(handler, 'onClick');

  const root = render(
    <svg>
      <DispenserMarker
        guid="test"
        location={[0, 0]}
        onClick={handler.onClick}
        data-testid="marker"
      />
    </svg>,
  );
  userEvent.click(root.getByTestId('marker'));
  expect(handler.onClick).toHaveBeenCalled();
});

it('smoke test - marker with image icon', () => {
  render(
    <svg>
      <DispenserMarker guid="test" location={[0, 0]} iconPath="/resources/ros-health.png" />
    </svg>,
  );
});
