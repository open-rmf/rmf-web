import { render, screen, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { LoopRequestForm } from '..';
import { availablePlaces, fleets } from './test-data';

describe('Form validation', () => {
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
    userEvent.type(root.getByPlaceholderText('Number of loops'), '1');
    userEvent.click(root.getByText('Request'));
    expect(doLoopRequest).toHaveBeenCalled();
  });

  test('Number of loops cannot be empty', () => {
    userEvent.click(root.getByText('Request'));
    expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(doLoopRequest).not.toHaveBeenCalled();
  });

  test('Start Location cannot be empty', () => {
    userEvent.type(root.getByPlaceholderText('Number of loops'), '1');
    userEvent.type(root.getByPlaceholderText('Pick Start Location'), '{selectall}{backspace}');
    userEvent.click(root.getByText('Request'));
    expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(doLoopRequest).not.toHaveBeenCalled();
  });

  test('Finish Location cannot be empty', () => {
    userEvent.type(root.getByPlaceholderText('Number of loops'), '1');
    userEvent.type(root.getByPlaceholderText('Pick Finish Location'), '{selectall}{backspace}');
    userEvent.click(root.getByText('Request'));
    expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(doLoopRequest).not.toHaveBeenCalled();
  });

  test('Start Location cannot be equal to Finish Location', () => {
    userEvent.type(root.getByPlaceholderText('Number of loops'), '1');
    userEvent.click(root.getByPlaceholderText('Pick Start Location'));
    userEvent.click(within(screen.getByRole('listbox')).getByText('placeA'));
    userEvent.click(root.getByPlaceholderText('Pick Finish Location'));
    userEvent.click(within(screen.getByRole('listbox')).getByText('placeA'));
    userEvent.click(root.getByText('Request'));

    expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(doLoopRequest).not.toHaveBeenCalled();
  });

  test('Changing target fleet updates available places', () => {
    userEvent.click(root.getByPlaceholderText('Choose Target Fleet'));
    userEvent.click(within(screen.getByRole('listbox')).getByText('fleetB'));
    userEvent.click(root.getByPlaceholderText('Pick Start Location'));

    expect(within(screen.getByRole('listbox')).getByText('placeC')).toBeTruthy();
  });
});
