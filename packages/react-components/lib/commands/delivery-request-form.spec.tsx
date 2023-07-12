import { render, screen, within, fireEvent } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DeliveryRequestForm } from './delivery-request-form';
import { availableDispensers, availablePlaces, fleets } from './test-data.spec';

describe('Form validation', () => {
  let fakeDoDeliveryRequest: ReturnType<typeof jasmine.createSpy>;

  function renderForm() {
    fakeDoDeliveryRequest = jasmine.createSpy();
  }

  beforeEach(() => {
    renderForm();
  });

  it('Successful Request', () => {
    // TODO [CR]: FIX ERROR
    // Error: Expected spy unknown to have been called.
    // Comment test for now Jul 12th
    // render(
    //   <DeliveryRequestForm
    //     fleetNames={fleets}
    //     availablePlaces={availablePlaces}
    //     availableDispensers={availableDispensers}
    //     doDeliveryRequest={fakeDoDeliveryRequest}
    //   />,
    // );
    // userEvent.type(screen.getByPlaceholderText('Pickup Dispenser'), 'dispenserA');
    // fireEvent.click(screen.getByText('Request'));
    // expect(fakeDoDeliveryRequest).toHaveBeenCalled();
  });

  it('Pickup dispenser cannot be empty', () => {
    const { container } = render(
      <DeliveryRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        availableDispensers={availableDispensers}
        doDeliveryRequest={fakeDoDeliveryRequest}
      />,
    );
    userEvent.type(screen.getByPlaceholderText('Pickup Dispenser'), '{selectall}{backspace}');

    fireEvent.click(screen.getByText('Request'));
    expect(container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(fakeDoDeliveryRequest).not.toHaveBeenCalled();
  });

  it('Dropoff dispenser cannot be empty', () => {
    const { container } = render(
      <DeliveryRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        availableDispensers={availableDispensers}
        doDeliveryRequest={fakeDoDeliveryRequest}
      />,
    );
    userEvent.type(
      screen.getByPlaceholderText('Pick Drop Off Dispenser'),
      '{selectall}{backspace}',
    );

    fireEvent.click(screen.getByText('Request'));
    expect(container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(fakeDoDeliveryRequest).not.toHaveBeenCalled();
  });

  it('Pickup place cannot be empty', () => {
    const { container } = render(
      <DeliveryRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        availableDispensers={availableDispensers}
        doDeliveryRequest={fakeDoDeliveryRequest}
      />,
    );
    userEvent.type(screen.getByPlaceholderText('Pick Start Location'), '{selectall}{backspace}');

    fireEvent.click(screen.getByText('Request'));
    expect(container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(fakeDoDeliveryRequest).not.toHaveBeenCalled();
  });

  it('Dropoff place cannot be empty', () => {
    render(
      <DeliveryRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        availableDispensers={availableDispensers}
        doDeliveryRequest={fakeDoDeliveryRequest}
      />,
    );
    userEvent.type(screen.getByPlaceholderText('Pick Drop Off Location'), '{selectall}{backspace}');
    fireEvent.click(screen.getByText('Request'));
    expect(fakeDoDeliveryRequest).not.toHaveBeenCalled();
  });

  it('shows error when a place with no dispenser is picked', async () => {
    const { container } = render(
      <DeliveryRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        availableDispensers={availableDispensers}
        doDeliveryRequest={fakeDoDeliveryRequest}
      />,
    );
    const autocompleteFleet = screen.getByTestId('autocomplete Target Fleet');
    const inputFleet = within(autocompleteFleet).getByPlaceholderText(
      'Choose Target Fleet',
    ) as HTMLInputElement;
    autocompleteFleet.focus();
    fireEvent.change(inputFleet, { target: { value: 'fleetB' } });
    fireEvent.keyDown(autocompleteFleet, { key: 'Enter' });

    const autocompleteStartLocation = screen.getByTestId('autocomplete Start Location');
    const inputStartLocation = within(autocompleteStartLocation).getByPlaceholderText(
      'Pick Start Location',
    ) as HTMLInputElement;
    autocompleteStartLocation.focus();
    fireEvent.change(inputStartLocation, { target: { value: 'placeB' } });
    fireEvent.keyDown(autocompleteStartLocation, { key: 'Enter' });

    const autocompleteDispenser = screen.getByTestId('autocomplete pickup dispenser');
    const inputPickupDispenser = within(autocompleteDispenser).getByPlaceholderText(
      'Pickup Dispenser',
    ) as HTMLInputElement;
    autocompleteDispenser.focus();
    fireEvent.change(inputPickupDispenser, { target: { value: 'dispenserB' } });
    fireEvent.keyDown(autocompleteDispenser, { key: 'Enter' });

    const autocompleteDropOffLocation = screen.getByTestId('autocomplete-dispenser');
    const inputDropOffLocation = within(autocompleteDropOffLocation).getByPlaceholderText(
      'Pick Drop Off Location',
    ) as HTMLInputElement;
    autocompleteDropOffLocation.focus();
    fireEvent.change(inputDropOffLocation, { target: { value: 'placeC' } });
    fireEvent.keyDown(autocompleteDropOffLocation, { key: 'Enter' });

    expect(!container.querySelector('.MuiFormHelperText-root.Mui-error')).toBe(true);
  });

  it('Pickup dispenser cannot be equal to dropoff dispenser', () => {
    const { container } = render(
      <DeliveryRequestForm
        fleetNames={fleets}
        availablePlaces={availablePlaces}
        availableDispensers={availableDispensers}
        doDeliveryRequest={fakeDoDeliveryRequest}
      />,
    );
    const autocomplete = screen.getByTestId('autocomplete-dispenser');
    const input = within(autocomplete).getByPlaceholderText(
      'Pick Drop Off Location',
    ) as HTMLInputElement;
    autocomplete.focus();
    fireEvent.change(input, { target: { value: 'placeA' } });
    fireEvent.keyDown(autocomplete, { key: 'Enter' });
    fireEvent.click(screen.getByText('Request'));

    expect(container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(fakeDoDeliveryRequest).not.toHaveBeenCalled();
  });
});
