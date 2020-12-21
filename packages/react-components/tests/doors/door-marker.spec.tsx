import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DoorMarker } from '../../lib';
import { mockOnClick } from '../test-utils';
import { makeDoor } from './test-utils';

it('triggers onClick callback when button is clicked', () => {
  const handler = mockOnClick();
  spyOn(handler, 'onClick');
  const root = render(
    <svg>
      <DoorMarker door={makeDoor()} onClick={handler.onClick} data-testid="marker" />
    </svg>,
  );
  userEvent.click(root.getByTestId('marker'));
  expect(handler.onClick).toHaveBeenCalled();
});
