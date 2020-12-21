import { render, screen, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { LoopRequestForm } from '../../lib';
import { availablePlaces, fleets } from './test-data';

describe('Form validation', () => {
  let handler: { doLoopRequest: () => void };
  let root: ReturnType<typeof renderForm>;

  function renderForm() {
    handler = {
      doLoopRequest: function doLoopRequest() {},
    };

    spyOn(handler, 'doLoopRequest');

    return render(
      <LoopRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        doLoopRequest={handler.doLoopRequest}
      />,
    );
  }

  beforeEach(() => {
    root = renderForm();
  });

  it('Successful Request', () => {
    userEvent.type(root.getByPlaceholderText('Number of loops'), '1');
    userEvent.click(root.getByText('Request'));
    expect(handler.doLoopRequest).toHaveBeenCalled();
  });

  it('Number of loops cannot be empty', () => {
    userEvent.click(root.getByText('Request'));
    expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(handler.doLoopRequest).not.toHaveBeenCalled();
  });

  it('Start Location cannot be empty', () => {
    userEvent.type(root.getByPlaceholderText('Number of loops'), '1');
    userEvent.type(root.getByPlaceholderText('Pick Start Location'), '{selectall}{backspace}');
    userEvent.click(root.getByText('Request'));
    expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(handler.doLoopRequest).not.toHaveBeenCalled();
  });

  it('Finish Location cannot be empty', () => {
    userEvent.type(root.getByPlaceholderText('Number of loops'), '1');
    userEvent.type(root.getByPlaceholderText('Pick Finish Location'), '{selectall}{backspace}');
    userEvent.click(root.getByText('Request'));
    expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(handler.doLoopRequest).not.toHaveBeenCalled();
  });

  it('Start Location cannot be equal to Finish Location', () => {
    userEvent.type(root.getByPlaceholderText('Number of loops'), '1');
    userEvent.click(root.getByPlaceholderText('Pick Start Location'));
    userEvent.click(within(screen.getByRole('listbox')).getByText('placeA'));
    userEvent.click(root.getByPlaceholderText('Pick Finish Location'));
    userEvent.click(within(screen.getByRole('listbox')).getByText('placeA'));
    userEvent.click(root.getByText('Request'));

    expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(handler.doLoopRequest).not.toHaveBeenCalled();
  });

  it('Changing target fleet updates available places', () => {
    userEvent.click(root.getByPlaceholderText('Choose Target Fleet'));
    userEvent.click(within(screen.getByRole('listbox')).getByText('fleetB'));
    userEvent.click(root.getByPlaceholderText('Pick Start Location'));

    expect(within(screen.getByRole('listbox')).getByText('placeC')).toBeTruthy();
  });
});
