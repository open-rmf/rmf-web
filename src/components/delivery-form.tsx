import { TextField, Button } from '@material-ui/core';
import Autocomplete from '@material-ui/lab/Autocomplete';
import React, { useState, Dispatch, SetStateAction, useEffect } from 'react';
import { loopFormStyles } from './loop-form';
import { TDeliveryRequest } from './commands-panel';
import { RobotResourceManager } from '../resource-manager-robots';

interface DeliveryFormProps {
  fleetNames: string[];
  requestDelivery: TDeliveryRequest;
  robotHandler: RobotResourceManager;
}

export const RobotDeliveryForm = (props: DeliveryFormProps): React.ReactElement => {
  const { requestDelivery, fleetNames, robotHandler } = props;
  const classes = loopFormStyles();

  const [targetFleetName, setTargetFleetName] = useState(
    fleetNames.length >= 1 ? fleetNames[0] : '',
  );
  const [listOfPlaces, setListOfPlaces] = useState(
    !!targetFleetName ? robotHandler.getAvailablePlacesPerFleet(targetFleetName) : [],
  );

  useEffect(() => {
    setListOfPlaces(robotHandler.getAvailablePlacesPerFleet(targetFleetName));
  }, [targetFleetName, robotHandler]);

  // Places
  const [pickupPlaceName, setPickupPlaceName] = useState(
    !!listOfPlaces && listOfPlaces.length >= 2 ? listOfPlaces[0] : '',
  );
  const [dropOffPlaceName, setDropOffPlaceName] = useState(
    !!listOfPlaces && listOfPlaces.length >= 2 ? listOfPlaces[1] : '',
  );

  // Dispensers
  const [pickupDispenser, setPickupDispenser] = useState('');
  const [dropOffDispenser, setDropOffDispenser] = useState('');

  // Error states
  const [targetFleetNameError, setTargetFleetNameError] = useState('');
  const [pickupPlaceNameError, setPickupPlaceNameError] = useState('');
  const [pickupDispenserError, setPickupDispenserError] = useState('');
  const [dropOffPlaceNameError, setDropOffPlaceNameError] = useState('');
  const [dropOffDispenserError, setDropOffDispenserError] = useState('');

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

  const handleDeliveryRequest = (event: React.FormEvent<HTMLFormElement>): void => {
    event.preventDefault();
    if (isFormValid()) {
      requestDelivery(pickupPlaceName, pickupDispenser, dropOffPlaceName, dropOffDispenser);
      cleanUpForm();
    }
  };

  const dispensersFromPickUpPlace = React.useMemo(() => {
    const dispenser = !!pickupPlaceName
      ? robotHandler.getDispensersPerFleet(targetFleetName, pickupPlaceName)
      : [];
    return !!dispenser ? dispenser : [];
  }, [pickupPlaceName, robotHandler, targetFleetName]);

  const dispensersFromDropOffPlace = React.useMemo(() => {
    const dispenser = !!dropOffPlaceName
      ? robotHandler.getDispensersPerFleet(targetFleetName, dropOffPlaceName)
      : [];
    return !!dispenser ? dispenser : [];
  }, [dropOffPlaceName, robotHandler, targetFleetName]);

  useEffect(() => {
    setPickupDispenserError('');
    !!dispensersFromPickUpPlace &&
      dispensersFromPickUpPlace.length === 0 &&
      setPickupDispenserError('There is no dispensers on this place. Pick another place');
  }, [dispensersFromPickUpPlace]);

  useEffect(() => {
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
          onChange={(e, value) => setTargetFleetName(value || fleetNames[0])}
          options={fleetNames}
          id={'deliveryFleet'}
          renderInput={params => (
            <TextField
              {...params}
              label="Choose Target Fleet"
              variant="outlined"
              error={!!targetFleetNameError}
              helperText={targetFleetNameError}
              name="targetFleet"
            />
          )}
          value={!!targetFleetName ? targetFleetName : null}
        />
      </div>

      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option}
          onChange={(e, value) => setPickupPlaceName(value || '')}
          options={listOfPlaces ? listOfPlaces : []}
          id={'pickupPlace'}
          renderInput={params => (
            <TextField
              {...params}
              error={!!pickupPlaceNameError}
              helperText={pickupPlaceNameError}
              label="Pick Start Location"
              name="pickupPlace"
              variant="outlined"
            />
          )}
          value={!!pickupPlaceName ? pickupPlaceName : null}
        />
      </div>

      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option}
          onChange={(e, value) => setPickupDispenser(value || '')}
          options={dispensersFromPickUpPlace}
          id={'pickupDispenser'}
          renderInput={params => (
            <TextField
              {...params}
              error={!!pickupDispenserError}
              helperText={pickupDispenserError}
              label="Pickup Dispenser"
              name="pickupDispenser"
              variant="outlined"
            />
          )}
          value={!!pickupDispenser ? pickupDispenser : null}
        />
      </div>

      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option}
          onChange={(e, value) => setDropOffPlaceName(value || '')}
          options={listOfPlaces ? listOfPlaces : []}
          id="dropoffPlace"
          renderInput={params => (
            <TextField
              {...params}
              error={!!dropOffPlaceNameError}
              helperText={dropOffPlaceNameError}
              label="Pick Drop Off Location"
              name="dropoffPlace"
              variant="outlined"
            />
          )}
          value={!!dropOffPlaceName ? dropOffPlaceName : null}
        />
      </div>

      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option}
          onChange={(e, value) => setDropOffDispenser(value || '')}
          options={dispensersFromDropOffPlace}
          id="dropoffDispenser"
          renderInput={params => (
            <TextField
              {...params}
              error={!!dropOffDispenserError}
              helperText={dropOffDispenserError}
              label="Pick Drop Off Dispenser"
              name="dropoffDispenser"
              variant="outlined"
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
