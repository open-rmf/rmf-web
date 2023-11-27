import { Autocomplete, styled } from '@mui/material';
import TextField from '@mui/material/TextField';
import React from 'react';
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
const StyledConfirmationDialog = styled((props: ConfirmationDialogProps) => (
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

export interface LiftRequestDialogProps {
  currentLevel: string;
  availableLevels: string[];
  showFormDialog: boolean;
  availableRequestTypes: number[];
  availableDoorModes: number[];
  onRequestSubmit?(
    event: React.FormEvent,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
  onClose: () => void;
}

export const LiftRequestDialog = ({
  currentLevel,
  availableLevels,
  availableRequestTypes,
  availableDoorModes,
  showFormDialog,
  onRequestSubmit,
  onClose,
}: LiftRequestDialogProps): JSX.Element => {
  const [doorState, setDoorState] = React.useState<number | null>(availableDoorModes[0]);
  const [requestType, setRequestType] = React.useState<number | null>(availableRequestTypes[0]);
  const [destination, setDestination] = React.useState(currentLevel);

  // Error states
  const [doorStateError, setDoorStateError] = React.useState('');
  const [requestTypeError, setRequestTypeError] = React.useState('');
  const [destinationError, setDestinationError] = React.useState('');

  const cleanUpError = () => {
    setDoorStateError('');
    setRequestTypeError('');
    setDestinationError('');
  };

  const isFormValid = () => {
    let isValid = true;
    cleanUpError();
    if (requestType === null) {
      setRequestTypeError('Request type cannot be empty');
      isValid = false;
    }
    if (doorState === null) {
      setDoorStateError('Door state cannot be empty');
      isValid = false;
    }
    if (!destination) {
      setDestinationError('Destination cannot be empty');
      isValid = false;
    }

    return isValid;
  };

  const handleLiftRequest = (event: React.FormEvent) => {
    event.preventDefault();
    if (isFormValid()) {
      onRequestSubmit &&
        doorState !== null &&
        requestType !== null &&
        onRequestSubmit(event, doorState, requestType, destination);
      onClose();
    }
  };

  return (
    <StyledConfirmationDialog
      open={showFormDialog}
      onClose={() => onClose()}
      fullWidth={true}
      maxWidth={'md'}
      onSubmit={handleLiftRequest}
      title={'Lift Request Form'}
      confirmText={'Request'}
      cancelText={'Close'}
    >
      <div className={classes.divForm}>
        <Autocomplete
          getOptionLabel={(option) => option}
          onChange={(_, value) => setDestination(value || '')}
          options={availableLevels}
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
          onChange={(_, value) => setRequestType(value as number)}
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
    </StyledConfirmationDialog>
  );
};

export default LiftRequestDialog;
