import { TextField, Button } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Autocomplete from '@material-ui/lab/Autocomplete';
import React, { useState } from 'react';
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
  const [dropOffPlaceNameError, setDropOffPlaceNameError] = useState('');

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
  };

  const handlerequestDelivery = (event: any) => {
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

    if (!pickupPlaceName) {
      setPickupPlaceNameError('Location cannot be empty');
      isValid = false;
    }
    if (!dropOffPlaceName) {
      setDropOffPlaceNameError('Location cannot be empty');
      isValid = false;
    }

    return isValid;
  };

  return (
    <form className={classes.form} onSubmit={handlerequestDelivery}>
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
          onChange={(e, value) => setPickupPlaceName(value || '')}
          options={listOfPlaces}
          renderInput={params => (
            <TextField {...params} label="Pick up Dispenser" variant="outlined" />
          )}
          value={!!pickupPlaceName ? pickupPlaceName : null}
        />
      </div>
      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option}
          onChange={(e, value) => setPickupPlaceName(value || '')}
          options={listOfPlaces}
          renderInput={params => (
            <TextField {...params} label="Pick Start Location" variant="outlined" />
          )}
          value={!!pickupPlaceName ? pickupPlaceName : null}
        />
      </div>

      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option}
          onChange={(e, value) => setDropOffPlaceName(value || '')}
          options={listOfPlaces}
          renderInput={params => (
            <TextField {...params} label="Pick Finish Location" variant="outlined" />
          )}
          value={!!dropOffPlaceName ? dropOffPlaceName : null}
        />
        {dropOffPlaceNameError && (
          <p id="dropOffPlaceNameError" className={classes.error}>
            {dropOffPlaceNameError}
          </p>
        )}
      </div>

      <div className={classes.buttonContainer}>
        <Button variant="contained" color="primary" type="submit" className={classes.button}>
          {'Request'}
        </Button>
      </div>
    </form>
  );
};
