import { makeStyles, TextField, Button, Input } from '@material-ui/core';
import Autocomplete from '@material-ui/lab/Autocomplete';
import React, { useState, useEffect } from 'react';
import { successMsg } from '../util/alerts';
import fakePlaces from '../mock/data/places';

interface robotLoopFormProps {
  fleetName: string;
  requestLoop: any;
}

export const RobotLoopForm = (props: robotLoopFormProps) => {
  const { requestLoop, fleetName } = props;
  const classes = useStyles();

  const [listOfPlacesToGo, setListOfPlacesToGo] = useState(['']);

  useEffect(() => {
    const listOfPlaces = fakePlaces()[fleetName];
    setListOfPlacesToGo(listOfPlaces);
    !listOfPlacesToGo && console.error('List of places to go it`s empty');
    // eslint-disable-next-line
  }, []);

  const [numLoops, setNumLoops] = useState(0);
  const [startLocation, setStartLocation] = useState(listOfPlacesToGo[0]);
  const [finishLocation, setFinishLocation] = useState(listOfPlacesToGo[1]);

  const [numLoopsError, setNumLoopsError] = useState('');
  const [startLocationError, setStartLocationError] = useState('');
  const [finishLocationError, setFinishLocationError] = useState('');

  const cleanUpForm = () => {
    setNumLoops(0);
    setStartLocation('');
    setFinishLocation('');
    cleanUpError();
  };

  const cleanUpError = () => {
    setNumLoopsError('');
    setStartLocationError('');
    setFinishLocationError('');
  };

  const handleRequestLoop = (event: any) => {
    event.preventDefault();
    if (isformValid()) {
      requestLoop(fleetName, numLoops, startLocation, finishLocation);
      successMsg('Success');
      cleanUpForm();
    }
  };

  const isformValid = () => {
    let isValid = true;
    cleanUpError();
    if (numLoops === 0 || numLoops < 0) {
      setNumLoopsError('Loops can only be > 0');
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
      <h4>Loops </h4>
      <div className={classes.divForm}>
        <Input
          name="numLoops"
          onChange={e => e.target.value && setNumLoops(parseInt(e.target.value))}
          placeholder="Number of loops"
          type="number"
          value={numLoops}
          className={classes.input}
        />
        {numLoopsError && <p className={classes.error}>{numLoopsError}</p>}
      </div>

      <h4>Start Location </h4>
      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option}
          onChange={(e, value) => setStartLocation(value || '')}
          options={listOfPlacesToGo}
          renderInput={params => <TextField {...params} label="Pick a place" variant="outlined" />}
          value={!!startLocation ? startLocation : null}
        />
        {startLocationError && <p className={classes.error}>{startLocationError}</p>}
      </div>

      <h4>Finish Location </h4>
      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option}
          onChange={(e, value) => setFinishLocation(value || '')}
          options={listOfPlacesToGo}
          renderInput={params => <TextField {...params} label="Pick a place" variant="outlined" />}
          value={!!finishLocation ? finishLocation : null}
        />
        {finishLocationError && <p className={classes.error}>{finishLocationError}</p>}
      </div>

      <div className={classes.buttonContainer}>
        <Button variant="contained" color="primary" type="submit">
          {'Request'}
        </Button>
      </div>
    </form>
  );
};

const useStyles = makeStyles(theme => ({
  form: {
    padding: '0.5rem',
    display: 'flex',
    flexDirection: 'column',
  },
  divForm: {
    padding: '0.2rem',
    width: '100%',
  },
  error: {
    color: 'red',
  },
  input: {
    width: '100%',
  },
  buttonContainer: {
    alignSelf: 'center',
  },
}));
