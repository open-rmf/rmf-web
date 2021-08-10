import { makeStyles, Dialog, IconButton, DialogContent } from '@material-ui/core';
import Button from '@material-ui/core/Button';
import TextField from '@material-ui/core/TextField';
import Autocomplete from '@material-ui/lab/Autocomplete';
import CloseIcon from '@material-ui/icons/Close';
import * as RmfModels from 'rmf-models';
import React from 'react';
import { requestDoorModeToString, requestModeToString } from './lift-utils';

const useStyles = makeStyles((theme) => ({
  closeButton: {
    position: 'absolute',
    right: theme.spacing(1),
    top: theme.spacing(1),
    color: theme.palette.error.main,
  },
  form: {
    display: 'flex',
    alignItems: 'center',
    flexDirection: 'column',
    padding: '0.5rem',
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
    width: '100%',
  },
  dialogContent: {
    padding: theme.spacing(5),
  },
}));

export interface LiftRequestFormProps {
  lift: RmfModels.Lift;
  showFormDialog: boolean;
  availableRequestTypes: number[];
  availableDoorModes: number[];
  onRequestSubmit?(
    event: React.FormEvent,
    lift: RmfModels.Lift,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
  onClose: () => void;
}

export const LiftRequestForm = (props: LiftRequestFormProps): JSX.Element => {
  const {
    lift,
    availableRequestTypes,
    availableDoorModes,
    showFormDialog,
    onRequestSubmit,
    onClose,
  } = props;
  const classes = useStyles();

  const [doorState, setDoorState] = React.useState(availableDoorModes[0]);
  const [requestType, setRequestType] = React.useState(availableRequestTypes[0]);
  const [destination, setDestination] = React.useState(lift.levels[0]);

  // Error states
  const [doorStateError, setDoorStateError] = React.useState('');
  const [requestTypeError, setRequestTypeError] = React.useState('');
  const [destinationError, setDestinationError] = React.useState('');

  const cleanUpForm = () => {
    setDoorState(availableDoorModes[0]);
    setRequestType(availableRequestTypes[0]);
    setDestination(lift.levels[0]);
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

  const handleLiftRequest = (event: React.FormEvent) => {
    event.preventDefault();
    if (isFormValid()) {
      onRequestSubmit && onRequestSubmit(event, lift, doorState, requestType, destination);
      cleanUpForm();
    }
  };

  return (
    <Dialog open={showFormDialog} onClose={() => onClose()} fullWidth={true} maxWidth={'md'}>
      <IconButton aria-label="close" className={classes.closeButton} onClick={() => onClose()}>
        <CloseIcon />
      </IconButton>
      <DialogContent className={classes.dialogContent} dividers>
        <form className={classes.form} onSubmit={handleLiftRequest}>
          <div className={classes.divForm}>
            <Autocomplete
              getOptionLabel={(option) => option}
              onChange={(_, value) => setDestination(value || '')}
              options={['', ...lift.levels]}
              renderInput={(params) => (
                <TextField
                  {...params}
                  label="Pick a Destination"
                  placeholder="Pick a Destination"
                  variant="outlined"
                  error={!!destinationError}
                  helperText={destinationError}
                />
              )}
              value={destination}
            />
          </div>

          <div className={classes.divForm}>
            <Autocomplete
              getOptionLabel={(option) => requestDoorModeToString(option)}
              onChange={(_, value) => setDoorState(value as number)}
              options={availableDoorModes}
              renderInput={(params) => (
                <TextField
                  {...params}
                  label="Pick a Door State"
                  placeholder="Pick a Door State"
                  variant="outlined"
                  error={!!doorStateError}
                  helperText={doorStateError}
                />
              )}
              value={doorState}
            />
          </div>

          <div className={classes.divForm}>
            <Autocomplete
              getOptionLabel={(option) => requestModeToString(option)}
              onChange={(_, value) => setRequestType((value as number) || availableRequestTypes[0])}
              options={availableRequestTypes}
              renderInput={(params) => (
                <TextField
                  {...params}
                  label="Pick Request Type"
                  placeholder="Pick Request Type"
                  variant="outlined"
                  error={!!requestTypeError}
                  helperText={requestTypeError}
                />
              )}
              value={requestType}
            />
          </div>

          <div className={classes.buttonContainer}>
            <Button variant="contained" color="primary" type="submit" className={classes.button}>
              Request
            </Button>
          </div>
        </form>
      </DialogContent>
    </Dialog>
  );
};

export default LiftRequestForm;
