import { makeStyles, TextField, Button } from '@material-ui/core';
import { successMsg } from '../../util/alerts';
import Autocomplete from '@material-ui/lab/Autocomplete';
import React, { useState } from 'react';

interface LiftRequestFormProps {
  requestTypeList: { key: string; value: number }[];
  doorStateList: { key: string; value: number }[];
  destinationList: string[];
  liftRequest(doorState: number, requestType: number, destination: string): void;
}

const LiftRequestForm = (props: LiftRequestFormProps) => {
  const { liftRequest, requestTypeList, doorStateList, destinationList } = props;
  const classes = useStyles();
  const [doorState, setDoorState] = useState(doorStateList[0]);
  const [requestType, setRequestType] = useState(requestTypeList[0]);
  const [destination, setDestination] = useState('');

  // Error states
  const [doorStateError, setDoorStateError] = useState('');
  const [requestTypeError, setRequestTypeError] = useState('');
  const [destinationError, setDestinationError] = useState('');

  const cleanUpForm = () => {
    setDoorState(doorStateList[0]);
    setRequestType(requestTypeList[0]);
    setDestination('');
    cleanUpError();
  };

  const cleanUpError = () => {
    setDoorStateError('');
    setRequestTypeError('');
    setDestinationError('');
  };

  const isFormValid = () => {
    let isValid = true;
    cleanUpError();
    if (!destination) {
      setDestinationError('Destination cannot be empty');
      isValid = false;
    }

    return isValid;
  };

  const handleLiftRequest = (event: any) => {
    event.preventDefault();
    if (isFormValid()) {
      liftRequest(doorState.value, requestType.value, destination);
      liftRequest(1, 1, destination);
      successMsg('Success');
      cleanUpForm();
    }
  };

  return (
    <form className={classes.form} onSubmit={handleLiftRequest}>
      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option}
          onChange={(e, value) => setDestination(value || '')}
          options={destinationList}
          renderInput={params => (
            <TextField {...params} label="Pick a Destination" variant="outlined" />
          )}
          value={!!destination ? destination : null}
        />
        {destinationError && (
          <p id="destinationError" className={classes.error}>
            {destinationError}
          </p>
        )}
      </div>

      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option.key}
          onChange={(e, value) => setDoorState(value || doorStateList[0])}
          options={doorStateList}
          renderInput={params => (
            <TextField {...params} label="Pick a Door State" variant="outlined" />
          )}
          value={!!doorState ? doorState : null}
        />
        {doorStateError && (
          <p id="doorStateError" className={classes.error}>
            {doorStateError}
          </p>
        )}
      </div>

      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={option => option.key}
          onChange={(e, value) => setRequestType(value || requestTypeList[0])}
          options={requestTypeList}
          renderInput={params => (
            <TextField {...params} label="Pick Request Type" variant="outlined" />
          )}
          value={!!requestType ? requestType : null}
        />
        {requestTypeError && (
          <p id="requestTypeError" className={classes.error}>
            {requestTypeError}
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

export default LiftRequestForm;

const useStyles = makeStyles(theme => ({
  form: {
    padding: '0.5rem',
    display: 'flex',
    flexDirection: 'column',
  },
  divForm: {
    padding: '0.5rem',
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
