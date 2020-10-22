import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { LoopRequestForm } from '..';

describe('Form validation', () => {
  const fleets = ['fleetA', 'fleetB'];

  const availablePlaces = {
    fleetA: ['placeA', 'placeB'],
    fleetB: ['placeB', 'placeC'],
  };

  let doLoopRequest: ReturnType<typeof jest.fn>;
  let root: ReturnType<typeof renderForm>;

  function renderForm() {
    doLoopRequest = jest.fn();
    return render(
      <LoopRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        doLoopRequest={doLoopRequest}
      />,
    );
  }

  beforeEach(() => {
    root = renderForm();
  });

  test('Successful Request', () => {
    userEvent.type(root.container.querySelector('[data-testid=numLoops] input')!, '1');
    userEvent.click(root.container.querySelector('button[type=submit]')!);
    expect(doLoopRequest).toHaveBeenCalled();
  });

  test('Number of loops cannot be empty', () => {
    userEvent.click(root.container.querySelector('button[type=submit]')!);
    expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(doLoopRequest).not.toHaveBeenCalled();
  });

  test('Start Location cannot be empty', () => {
    userEvent.type(root.container.querySelector('[data-testid=numLoops] input')!, '1');
    userEvent.type(
      root.container.querySelector('[data-testid=startLocation] input')!,
      '{selectall}{backspace}',
    );
    userEvent.click(root.container.querySelector('button[type=submit]')!);
    expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(doLoopRequest).not.toHaveBeenCalled();
  });

  test('Finish Location cannot be empty', () => {
    userEvent.type(root.container.querySelector('[data-testid=numLoops] input')!, '1');
    userEvent.type(
      root.container.querySelector('[data-testid=finishLocation] input')!,
      '{selectall}{backspace}',
    );
    userEvent.click(root.container.querySelector('button[type=submit]')!);
    expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(doLoopRequest).not.toHaveBeenCalled();
  });

  test('Start Location cannot be equal to Finish Location', () => {
    userEvent.type(root.container.querySelector('[data-testid=numLoops] input')!, '1');
    userEvent.click(root.container.querySelector('[data-testid=startLocation] input')!);
    userEvent.click(document.querySelectorAll('.MuiAutocomplete-option')[0]);
    userEvent.click(root.container.querySelector('[data-testid=finishLocation] input')!);
    userEvent.click(document.querySelectorAll('.MuiAutocomplete-option')[0]);
    userEvent.click(root.container.querySelector('button[type=submit]')!);

    expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(doLoopRequest).not.toHaveBeenCalled();
  });

  test('Changing target fleet updates available places', () => {
    userEvent.click(root.container.querySelector('[data-testid=targetFleet] input')!);
    userEvent.click(document.querySelectorAll('.MuiAutocomplete-option')[1]);
    userEvent.click(root.container.querySelector('[data-testid=startLocation] input')!);
    userEvent.click(document.querySelectorAll('.MuiAutocomplete-option')[1]);

    expect(
      root.container.querySelector('[data-testid=startLocation] input')!.getAttribute('value'),
    ).toBe('placeC');
  });
});
