import { render, screen, within, fireEvent } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { LoopRequestForm } from './loop-request-form';
import { availablePlaces, fleets } from './test-data.spec';

describe('Form validation', () => {
  let fakeDoLoopRequest: ReturnType<typeof jasmine.createSpy>;

  function renderForm() {
    fakeDoLoopRequest = jasmine.createSpy();
  }

  beforeEach(() => {
    renderForm();
  });

  it('Successful Request', () => {
    render(
      <LoopRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        doLoopRequest={fakeDoLoopRequest}
      />,
    );
    userEvent.type(screen.getByPlaceholderText('Number of loops'), '1');
    fireEvent.click(screen.getByText('Request'));
    expect(fakeDoLoopRequest).toHaveBeenCalled();
  });

  it('Number of loops cannot be empty', () => {
    const { container } = render(
      <LoopRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        doLoopRequest={fakeDoLoopRequest}
      />,
    );
    fireEvent.click(screen.getByText('Request'));
    expect(container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(fakeDoLoopRequest).not.toHaveBeenCalled();
  });

  it('Start Location cannot be empty', () => {
    const { container } = render(
      <LoopRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        doLoopRequest={fakeDoLoopRequest}
      />,
    );
    userEvent.type(screen.getByPlaceholderText('Number of loops'), '1');
    userEvent.type(screen.getByPlaceholderText('Pick Start Location'), '{selectall}{backspace}');
    fireEvent.click(screen.getByText('Request'));
    expect(container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(fakeDoLoopRequest).not.toHaveBeenCalled();
  });

  it('Finish Location cannot be empty', () => {
    const { container } = render(
      <LoopRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        doLoopRequest={fakeDoLoopRequest}
      />,
    );
    userEvent.type(screen.getByPlaceholderText('Number of loops'), '1');
    userEvent.type(screen.getByPlaceholderText('Pick Finish Location'), '{selectall}{backspace}');
    fireEvent.click(screen.getByText('Request'));
    expect(container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(fakeDoLoopRequest).not.toHaveBeenCalled();
  });

  it('Start Location cannot be equal to Finish Location', () => {
    const { container } = render(
      <LoopRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        doLoopRequest={fakeDoLoopRequest}
      />,
    );
    const autocomplete = screen.getByTestId('autocomplete-location');
    const input = within(autocomplete).getByPlaceholderText(
      'Pick Start Location',
    ) as HTMLInputElement;
    autocomplete.focus();
    fireEvent.change(input, { target: { value: 'placeA' } });
    fireEvent.keyDown(autocomplete, { key: 'Enter' });
    fireEvent.click(screen.getByText('Request'));

    expect(container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(fakeDoLoopRequest).not.toHaveBeenCalled();
  });

  it('Changing target fleet updates available places', async () => {
    render(
      <LoopRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        doLoopRequest={fakeDoLoopRequest}
      />,
    );
    const autocomplete = screen.getByTestId('autocomplete-fleet');
    const input = within(autocomplete).getByPlaceholderText(
      'Choose Target Fleet',
    ) as HTMLInputElement;
    autocomplete.focus();
    fireEvent.change(input, { target: { value: 'fleetB' } });
    fireEvent.keyDown(autocomplete, { key: 'ArrowDown' });
    fireEvent.keyDown(autocomplete, { key: 'Enter' });
    expect(input.value).toEqual('fleetB');
  });
});
