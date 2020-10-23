import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DoorAccordion } from '..';
import { makeDoor } from './test-utils';

test('triggers doOpenDoor callback when button is clicked', () => {
  const handleDoorOpen = jest.fn();
  const root = render(<DoorAccordion door={makeDoor()} onOpenClick={handleDoorOpen} />);
  userEvent.click(root.getByText('Open'));
  expect(handleDoorOpen).toBeCalled();
});

test('triggers doCloseDoor callback when button is clicked', () => {
  const handleDoorClose = jest.fn();
  const root = render(<DoorAccordion door={makeDoor()} onCloseClick={handleDoorClose} />);
  userEvent.click(root.getByText('Close'));
  expect(handleDoorClose).toBeCalled();
});
