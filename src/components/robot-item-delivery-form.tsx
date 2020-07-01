import { TextField, Button } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Autocomplete from '@material-ui/lab/Autocomplete';
import React, { useState, Dispatch, SetStateAction } from 'react';
import { robotFormStyle } from './robot-item-loop-form';

interface robotDeliveryFormProps {
  fleetName: string;
  requestDelivery(
    pickupPlaceName: string,
    pickupDispenser: string,
    dropOffPlaceName: string,
    dropOffDispenser: string,
    pickupBehaviour?: RomiCore.Behavior,
    dropOffBehavior?: RomiCore.Behavior,
  ): void;
  listOfPlaces: string[];
}

export const RobotDeliveryForm = (props: robotDeliveryFormProps) => {
  const { requestDelivery, listOfPlaces } = props;
  const classes = robotFormStyle();
  // Pick up
  const [pickupPlaceName, setPickupPlaceName] = useState(
    listOfPlaces.length >= 2 ? listOfPlaces[0] : '',
  );
  const [pickupDispenser, setPickupDispenser] = useState('');
  // Drop off
  const [dropOffPlaceName, setDropOffPlaceName] = useState(
    listOfPlaces.length >= 2 ? listOfPlaces[1] : '',
  );
  const [dropOffDispenser, setDropOffDispenser] = useState('');

  // Error states
  const [pickupPlaceNameError, setPickupPlaceNameError] = useState('');
  const [pickupDispenserError, setPickupDispenserError] = useState('');
  const [dropOffPlaceNameError, setDropOffPlaceNameError] = useState('');
  const [dropOffDispenserError, setDropOffDispenserError] = useState('');

  const cleanUpForm = () => {
    setPickupPlaceName(listOfPlaces.length >= 2 ? listOfPlaces[0] : '');
    setDropOffPlaceName(listOfPlaces.length >= 2 ? listOfPlaces[1] : '');
    setPickupDispenser('');
    setDropOffDispenser('');
    cleanUpError();
  };

  const cleanUpError = () => {
    setPickupPlaceNameError('');
    setDropOffPlaceNameError('');
    setPickupDispenserError('');
    setDropOffDispenserError('');
  };

  const handleDeliveryRequest = (event: React.FormEvent<HTMLFormElement>): void => {
    event.preventDefault();
    if (isFormValid()) {
      requestDelivery(pickupPlaceName, pickupDispenser, dropOffPlaceName, dropOffDispenser);
      cleanUpForm();
    }
  };

  const isFormValid = () => {
    let isValid = true;
    cleanUpError();

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

    const setEmpty = (fieldSetter: Dispatch<SetStateAction<string>>): void => {
      fieldSetter('Cannot be empty');
      isValid = false;
    };

    !pickupPlaceName && setEmpty(setPickupPlaceNameError);
    !dropOffPlaceName && setEmpty(setDropOffPlaceNameError);
    !pickupDispenser && setEmpty(setPickupDispenserError);
    !dropOffDispenser && setEmpty(setDropOffDispenserError);

    return isValid;
  };

  return (
    <form className={classes.form} onSubmit={handleDeliveryRequest}>
      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option}
          onChange={(e, value) => setPickupPlaceName(value || '')}
          options={listOfPlaces}
          renderInput={params => (
            <TextField
              {...params}
              label="Pick Start Location"
              variant="outlined"
              error={!!pickupPlaceNameError}
              helperText={pickupPlaceNameError}
            />
          )}
          value={!!pickupPlaceName ? pickupPlaceName : null}
        />
      </div>

      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option}
          onChange={(e, value) => setPickupDispenser(value || '')}
          options={listOfPlaces}
          renderInput={params => (
            <TextField
              {...params}
              label="Pickup Dispenser"
              variant="outlined"
              error={!!pickupDispenserError}
              helperText={pickupDispenserError}
            />
          )}
          value={!!pickupDispenser ? pickupDispenser : null}
        />
      </div>

      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option}
          onChange={(e, value) => setDropOffPlaceName(value || '')}
          options={listOfPlaces}
          renderInput={params => (
            <TextField
              {...params}
              label="Pick Drop Off Location"
              variant="outlined"
              error={!!dropOffPlaceNameError}
              helperText={dropOffPlaceNameError}
            />
          )}
          value={!!dropOffPlaceName ? dropOffPlaceName : null}
        />
      </div>

      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option}
          onChange={(e, value) => setDropOffDispenser(value || '')}
          options={listOfPlaces}
          renderInput={params => (
            <TextField
              {...params}
              label="Pick Drop Off Dispenser"
              variant="outlined"
              error={!!dropOffDispenserError}
              helperText={dropOffDispenserError}
            />
          )}
          value={!!dropOffDispenser ? dropOffDispenser : null}
        />
      </div>

      <div className={classes.buttonContainer}>
        <Button variant="contained" color="primary" type="submit" className={classes.button}>
          {'Request'}
        </Button>
      </div>
    </form>
  );
};
