import { Button, TextField, Autocomplete } from '@mui/material';
import React, { ChangeEvent } from 'react';
import { StyledForm, commandFormsClasses } from './form-styles';

export type DoDeliveryRequest = (
  pickupPlaceName: string,
  pickupDispenser: string,
  dropOffPlaceName: string,
  dropOffDispenser: string,
) => void;

export interface DeliveryRequestFormProps {
  fleetNames: string[];
  availablePlaces(fleet: string): string[];
  availableDispensers(fleet: string, place: string): string[];
  doDeliveryRequest?: DoDeliveryRequest;
}

export const DeliveryRequestForm = React.forwardRef(
  (props: DeliveryRequestFormProps, ref: React.Ref<HTMLFormElement>): JSX.Element => {
    const { fleetNames, availableDispensers, availablePlaces, doDeliveryRequest } = props;

    const [targetFleetName, setTargetFleetName] = React.useState(
      fleetNames.length >= 1 ? fleetNames[0] : '',
    );
    const [listOfPlaces, setListOfPlaces] = React.useState(
      targetFleetName ? availablePlaces(targetFleetName) : [],
    );

    // Places
    const [pickupPlaceName, setPickupPlaceName] = React.useState(
      !!listOfPlaces && listOfPlaces.length >= 2 ? listOfPlaces[0] : '',
    );
    const [dropOffPlaceName, setDropOffPlaceName] = React.useState(
      !!listOfPlaces && listOfPlaces.length >= 2 ? listOfPlaces[1] : '',
    );

    // Dispensers
    const [pickupDispenser, setPickupDispenser] = React.useState('');
    const [dropOffDispenser, setDropOffDispenser] = React.useState('');

    // Error states
    const [targetFleetNameError, setTargetFleetNameError] = React.useState('');
    const [pickupPlaceNameError, setPickupPlaceNameError] = React.useState('');
    const [pickupDispenserError, setPickupDispenserError] = React.useState('');
    const [dropOffPlaceNameError, setDropOffPlaceNameError] = React.useState('');
    const [dropOffDispenserError, setDropOffDispenserError] = React.useState('');

    const cleanUpForm = (): void => {
      setTargetFleetName(fleetNames.length >= 1 ? fleetNames[0] : '');
      setPickupPlaceName(listOfPlaces && listOfPlaces.length >= 2 ? listOfPlaces[0] : '');
      setDropOffPlaceName(listOfPlaces && listOfPlaces.length >= 2 ? listOfPlaces[1] : '');
      setPickupDispenser('');
      setDropOffDispenser('');
      cleanUpError();
    };

    const cleanUpError = (): void => {
      setTargetFleetNameError('');
      setPickupPlaceNameError('');
      setDropOffPlaceNameError('');
      setPickupDispenserError('');
      setDropOffDispenserError('');
    };

    const handleSubmit = (ev: React.FormEvent): void => {
      ev.preventDefault();
      if (isFormValid()) {
        doDeliveryRequest &&
          doDeliveryRequest(pickupPlaceName, pickupDispenser, dropOffPlaceName, dropOffDispenser);
        cleanUpForm();
      }
    };

    const dispensersFromPickUpPlace = React.useMemo(() => {
      const dispenser = pickupPlaceName
        ? availableDispensers(targetFleetName, pickupPlaceName)
        : [];
      return dispenser ? dispenser : [];
    }, [pickupPlaceName, targetFleetName, availableDispensers]);

    const dispensersFromDropOffPlace = React.useMemo(() => {
      const dispenser = dropOffPlaceName
        ? availableDispensers(targetFleetName, dropOffPlaceName)
        : [];
      return dispenser ? dispenser : [];
    }, [dropOffPlaceName, targetFleetName, availableDispensers]);

    React.useEffect(() => {
      setPickupDispenserError('');
      !!dispensersFromPickUpPlace &&
        dispensersFromPickUpPlace.length === 0 &&
        setPickupDispenserError('There is no dispensers on this place. Pick another place');
    }, [dispensersFromPickUpPlace]);

    React.useEffect(() => {
      setDropOffDispenserError('');
      !!dispensersFromDropOffPlace &&
        dispensersFromDropOffPlace.length === 0 &&
        setDropOffDispenserError('There is no dispensers on this place. Pick another place');
    }, [dispensersFromDropOffPlace]);

    const isFormValid = (): boolean => {
      let isValid = true;
      cleanUpError();

      if (targetFleetName === '') {
        setTargetFleetNameError('Fleet name cannot be empty');
        isValid = false;
      }

      if (pickupPlaceName === dropOffPlaceName) {
        setPickupPlaceNameError('Start Location cannot be equal to finish Location');
        setDropOffPlaceNameError('Start Location cannot be equal to finish Location');
        isValid = false;
      }

      if (pickupPlaceName === dropOffPlaceName) {
        setPickupDispenserError('Pickup dispenser cannot be equal to Drop off dispenser');
        setDropOffDispenserError('Drop off dispenser cannot be equal to Pickup dispenser');
        isValid = false;
      }

      const setEmpty = (fieldSetter: React.Dispatch<React.SetStateAction<string>>): void => {
        fieldSetter('Cannot be empty');
        isValid = false;
      };

      !pickupPlaceName && setEmpty(setPickupPlaceNameError);
      !dropOffPlaceName && setEmpty(setDropOffPlaceNameError);
      !pickupDispenser && setEmpty(setPickupDispenserError);
      !dropOffDispenser && setEmpty(setDropOffDispenserError);

      return isValid;
    };

    const handleTargetFleetNameChange = (_: ChangeEvent<unknown>, value: string | null) => {
      const newFleetName = value || fleetNames[0];
      const newPlaces = availablePlaces(newFleetName);
      setPickupPlaceName((cur) => {
        if (newPlaces.includes(cur)) {
          return cur;
        }
        return newPlaces.length >= 2 ? newPlaces[0] : '';
      });
      setPickupDispenser('');
      setDropOffPlaceName((cur) => {
        if (newPlaces.includes(cur)) {
          return cur;
        }
        return newPlaces.length >= 2 ? newPlaces[1] : '';
      });
      setDropOffDispenser('');
      setListOfPlaces(availablePlaces(newFleetName));
      setTargetFleetName(newFleetName);
    };

    const handlePickupPlaceNameChange = (_: ChangeEvent<unknown>, value: string | null) => {
      setPickupPlaceName(value || '');
      setPickupDispenser('');
    };

    const handleDropOoffPlaceNameChange = (_: ChangeEvent<unknown>, value: string | null) => {
      setDropOffPlaceName(value || '');
      setDropOffDispenser('');
    };

    return (
      <StyledForm ref={ref} className={commandFormsClasses.form} onSubmit={handleSubmit}>
        <div className={commandFormsClasses.divForm}>
          <Autocomplete
            getOptionLabel={(option) => option}
            onChange={handleTargetFleetNameChange}
            options={fleetNames}
            renderInput={(params) => (
              <TextField
                {...params}
                label="Choose Target Fleet"
                placeholder="Choose Target Fleet"
                variant="outlined"
                error={!!targetFleetNameError}
                helperText={targetFleetNameError}
              />
            )}
            value={targetFleetName ? targetFleetName : null}
          />
        </div>

        <div className={commandFormsClasses.divForm}>
          <Autocomplete
            getOptionLabel={(option) => option}
            onChange={handlePickupPlaceNameChange}
            options={listOfPlaces ? listOfPlaces : []}
            renderInput={(params) => (
              <TextField
                {...params}
                error={!!pickupPlaceNameError}
                helperText={pickupPlaceNameError}
                label="Pick Start Location"
                placeholder="Pick Start Location"
                variant="outlined"
              />
            )}
            value={pickupPlaceName ? pickupPlaceName : null}
          />
        </div>

        <div className={commandFormsClasses.divForm}>
          <Autocomplete
            getOptionLabel={(option) => option}
            onChange={(_, value) => setPickupDispenser(value || '')}
            options={dispensersFromPickUpPlace}
            renderInput={(params) => (
              <TextField
                {...params}
                error={!!pickupDispenserError}
                helperText={pickupDispenserError}
                label="Pickup Dispenser"
                placeholder="Pickup Dispenser"
                variant="outlined"
              />
            )}
            value={pickupDispenser ? pickupDispenser : null}
          />
        </div>

        <div className={commandFormsClasses.divForm}>
          <Autocomplete
            getOptionLabel={(option) => option}
            onChange={handleDropOoffPlaceNameChange}
            options={listOfPlaces ? listOfPlaces : []}
            renderInput={(params) => (
              <TextField
                {...params}
                error={!!dropOffPlaceNameError}
                helperText={dropOffPlaceNameError}
                label="Pick Drop Off Location"
                placeholder="Pick Drop Off Location"
                variant="outlined"
              />
            )}
            value={dropOffPlaceName ? dropOffPlaceName : null}
          />
        </div>

        <div className={commandFormsClasses.divForm}>
          <Autocomplete
            getOptionLabel={(option) => option}
            onChange={(_, value) => setDropOffDispenser(value || '')}
            options={dispensersFromDropOffPlace}
            renderInput={(params) => (
              <TextField
                {...params}
                error={!!dropOffDispenserError}
                helperText={dropOffDispenserError}
                label="Pick Drop Off Dispenser"
                placeholder="Pick Drop Off Dispenser"
                variant="outlined"
              />
            )}
            value={dropOffDispenser ? dropOffDispenser : null}
          />
        </div>

        <div className={commandFormsClasses.buttonContainer}>
          <Button
            variant="contained"
            color="primary"
            type="submit"
            className={commandFormsClasses.button}
          >
            Request
          </Button>
        </div>
      </StyledForm>
    );
  },
);

export default DeliveryRequestForm;
