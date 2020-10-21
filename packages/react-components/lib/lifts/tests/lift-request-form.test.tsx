import { fireEvent, render } from '@testing-library/react';
import React from 'react';
import { LiftRequestForm } from '..';
import { requestDoorModes, requestModes } from '../lift-utils';
import { makeLift } from './test-utils';

function renderLiftRequestForm() {
  return render(
    <LiftRequestForm
      lift={makeLift()}
      availableRequestTypes={requestModes}
      availableDoorModes={requestDoorModes}
    />,
  );
}

let root: ReturnType<typeof renderLiftRequestForm>;

beforeEach(() => {
  root = render(
    <LiftRequestForm
      lift={makeLift()}
      availableRequestTypes={requestModes}
      availableDoorModes={requestDoorModes}
    />,
  );
});

test('resets form after submitting', () => {
  const initialDestinationHtml = root.getByTestId('destination').outerHTML;
  const initialDoorStateHtml = root.getByTestId('door-state').outerHTML;
  const initialRequestTypeHtml = root.getByTestId('request-type').outerHTML;

  fireEvent.submit(root.getByTestId('request-form'));

  expect(root.getByTestId('destination').outerHTML).toBe(initialDestinationHtml);
  expect(root.getByTestId('door-state').outerHTML).toBe(initialDoorStateHtml);
  expect(root.getByTestId('request-type').outerHTML).toBe(initialRequestTypeHtml);
});

test('destination is required', () => {
  const autocomplete = root.getByTestId('destination');
  const input = autocomplete.querySelector('input');
  if (!input) {
    fail('destination value element not found');
  }
  fireEvent.change(input, {
    target: { value: '' },
  });
  fireEvent.submit(root.getByTestId('request-form'));
  expect(root.container.querySelectorAll('[data-error]')).toHaveLength(1);
});
