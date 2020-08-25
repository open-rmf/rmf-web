import { makeStyles, TextField, Button } from '@material-ui/core';
import Autocomplete from '@material-ui/lab/Autocomplete';
import React, { useState, useEffect } from 'react';
import { successMsg } from '../util/alerts';
import { TLoopRequest } from './commands-panel';
import { ResourcesContext } from '../app-contexts';
interface LoopFormProps {
  fleetNames: string[];
  requestLoop: TLoopRequest;
}

export const LoopForm = (props: LoopFormProps) => {
  const { requestLoop, fleetNames } = props;
  const classes = loopFormStyles();
  const resourcesContext = React.useContext(ResourcesContext);

  const [targetFleetName, setTargetFleetName] = useState(
    fleetNames.length >= 1 ? fleetNames[0] : '',
  );
  const [numLoops, setNumLoops] = useState(0);
  const [listOfPlaces, setListOfPlaces] = useState(
    !!targetFleetName ? resourcesContext.robots.getAvailablePlacesPerFleet(targetFleetName) : [],
  );
  const [startLocation, setStartLocation] = useState(
    listOfPlaces && listOfPlaces.length >= 2 ? listOfPlaces[0] : '',
  );
  const [finishLocation, setFinishLocation] = useState(
    listOfPlaces && listOfPlaces.length >= 2 ? listOfPlaces[1] : '',
  );

  useEffect(() => {
    resourcesContext.robots.getAvailablePlacesPerFleet(targetFleetName)
      ? setListOfPlaces(resourcesContext.robots.getAvailablePlacesPerFleet(targetFleetName))
      : setListOfPlaces([]);
  }, [targetFleetName, resourcesContext]);

  useEffect(() => {
    if (listOfPlaces) {
      setStartLocation(listOfPlaces.length >= 2 ? listOfPlaces[0] : '');
      setFinishLocation(listOfPlaces.length >= 2 ? listOfPlaces[1] : '');
    }
  }, [listOfPlaces]);

  // Error states
  const [targetFleetNameError, setTargetFleetNameError] = useState('');
  const [numLoopsError, setNumLoopsError] = useState('');
  const [startLocationError, setStartLocationError] = useState('');
  const [finishLocationError, setFinishLocationError] = useState('');

  const cleanUpForm = () => {
    setTargetFleetName(targetFleetName);
    setNumLoops(0);
    setListOfPlaces(
      !!targetFleetName ? resourcesContext.robots.getAvailablePlacesPerFleet(targetFleetName) : [],
    );
    setStartLocation(listOfPlaces && listOfPlaces.length >= 2 ? listOfPlaces[0] : '');
    setFinishLocation(listOfPlaces && listOfPlaces.length >= 2 ? listOfPlaces[1] : '');
    cleanUpError();
  };

  const cleanUpError = () => {
    setTargetFleetNameError('');
    setNumLoopsError('');
    setStartLocationError('');
    setFinishLocationError('');
  };

  const handleRequestLoop = (event: any) => {
    event.preventDefault();
    if (isFormValid()) {
      requestLoop(targetFleetName, numLoops, startLocation, finishLocation);
      successMsg('Success');
      cleanUpForm();
    }
  };

  const isFormValid = () => {
    let isValid = true;
    cleanUpError();
    if (targetFleetName === '') {
      setTargetFleetNameError('Fleet name cannot be empty');
      isValid = false;
    }
    if (numLoops === 0 || numLoops < 0) {
      setNumLoopsError('Loops can only be > 0');
      isValid = false;
    }
    if (startLocation === finishLocation) {
      setStartLocationError('Start Location cannot be equal to Finish Location');
      setFinishLocationError('Start Location cannot be equal to Finish Location');
      isValid = false;
    }

    if (!startLocation) {
      setStartLocationError('Location cannot be empty');
      isValid = false;
    }
    if (!finishLocation) {
      setFinishLocationError('Location cannot be empty');
      isValid = false;
    }

    return isValid;
  };

  return (
    <form className={classes.form} onSubmit={handleRequestLoop}>
      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={(option) => option}
          onChange={(e, value) => setTargetFleetName(value || '')}
          options={fleetNames}
          renderInput={(params) => (
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
        <TextField
          id="numLoops"
          name="numLoops"
          onChange={(e) => {
            setNumLoops(!!e.target.value ? parseInt(e.target.value) : 0);
          }}
          placeholder="Number of loops"
          type="number"
          value={numLoops || ''}
          className={classes.input}
          label="Number of loops"
          variant="outlined"
          error={!!numLoopsError}
          helperText={numLoopsError}
        />
      </div>

      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={(option) => option}
          onChange={(e, value) => setStartLocation(value || '')}
          options={listOfPlaces ? listOfPlaces : []}
          id="startLocation"
          renderInput={(params) => (
            <TextField
              {...params}
              label="Pick Start Location"
              variant="outlined"
              error={!!startLocationError}
              helperText={startLocationError}
              name="startLocation"
            />
          )}
          value={!!startLocation ? startLocation : null}
        />
      </div>

      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={(option) => option}
          onChange={(e, value) => setFinishLocation(value || '')}
          options={listOfPlaces ? listOfPlaces : []}
          id="finishLocation"
          renderInput={(params) => (
            <TextField
              {...params}
              label="Pick Finish Location"
              variant="outlined"
              error={!!finishLocationError}
              helperText={finishLocationError}
              name="finishLocation"
            />
          )}
          value={!!finishLocation ? finishLocation : null}
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

export const loopFormStyles = makeStyles((theme) => ({
  form: {
    display: 'flex',
    flexDirection: 'column',
    width: '100%',
  },
  divForm: {
    padding: '0.46rem',
    paddingRight: '0.5rem',
    width: '100%',
  },
  error: {
    color: theme.palette.error.main,
  },
  input: {
    width: '100%',
  },
  button: {
    width: '100%',
  },
  buttonContainer: {
    paddingTop: '0.5rem',
    paddingLeft: '0.5rem',
    width: '100%',
  },
}));
