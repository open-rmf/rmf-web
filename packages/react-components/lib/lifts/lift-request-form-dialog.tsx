import { IconButton, styled } from '@material-ui/core';
import Button from '@material-ui/core/Button';
import TextField from '@material-ui/core/TextField';
import CloseIcon from '@material-ui/icons/Close';
import Autocomplete from '@material-ui/lab/Autocomplete';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { ConfirmationDialog, ConfirmationDialogProps } from '../confirmation-dialog';
import { requestDoorModeToString, requestModeToString } from './lift-utils';

const classes = {
  closeButton: 'lift-request-close-button',
  form: 'lift-request-form',
  divForm: 'lift-request-divform',
  error: 'lift-request-error',
  input: 'lift-request-input',
  button: 'lift-request-button',
  buttonContainer: 'lift-request-button-container',
  dialogContent: 'lift-request-dialog-content',
};
const LiftRequestFormDialogRoot = styled((props: ConfirmationDialogProps) => (
  <ConfirmationDialog {...props} />
))(({ theme }) => ({
  [`& .${classes.closeButton}`]: {
    position: 'absolute',
    right: theme.spacing(1),
    top: theme.spacing(1),
    color: theme.palette.error.main,
  },
  [`& .${classes.form}`]: {
    display: 'flex',
    alignItems: 'center',
    flexDirection: 'column',
    padding: '0.5rem',
  },
  [`& .${classes.divForm}`]: {
    padding: '0.5rem',
    width: '100%',
  },
  [`& .${classes.error}`]: {
    color: theme.palette.error.main,
  },
  [`& .${classes.input}`]: {
    width: '100%',
  },
  [`& .${classes.button}`]: {
    width: '100%',
  },
  [`& .${classes.buttonContainer}`]: {
    paddingTop: '0.5rem',
    width: '100%',
  },
  [`& .${classes.dialogContent}`]: {
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

export const LiftRequestFormDialog = ({
  lift,
  availableRequestTypes,
  availableDoorModes,
  showFormDialog,
  onRequestSubmit,
  onClose,
}: LiftRequestFormProps): JSX.Element => {
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
    <LiftRequestFormDialogRoot
      open={showFormDialog}
      onClose={() => onClose()}
      fullWidth={true}
      maxWidth={'md'}
      onSubmit={handleLiftRequest}
    >
      <IconButton aria-label="close" className={classes.closeButton} onClick={() => onClose()}>
        <CloseIcon />
      </IconButton>
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
    </LiftRequestFormDialogRoot>
  );
};

export default LiftRequestFormDialog;
