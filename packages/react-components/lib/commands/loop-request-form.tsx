import { Button, TextField, Autocomplete } from '@mui/material';
import React, { ChangeEvent } from 'react';
import { StyledForm, commandFormsClasses } from './form-styles';

export type DoLoopRequest = (
  fleetName: string,
  numLoops: number,
  startLocationPoint: string,
  endLocationPoint: string,
) => void;

export interface LoopRequestFormProps {
  fleetNames: string[];
  availablePlaces(fleet: string): string[];
  doLoopRequest?: DoLoopRequest;
}

export const LoopRequestForm = React.forwardRef(
  (props: LoopRequestFormProps, ref: React.Ref<HTMLFormElement>): JSX.Element => {
    const { fleetNames, availablePlaces, doLoopRequest } = props;

    const [targetFleetName, setTargetFleetName] = React.useState(
      fleetNames.length >= 1 ? fleetNames[0] : '',
    );

    const [numLoops, setNumLoops] = React.useState(0);
    const [listOfPlaces, setListOfPlaces] = React.useState(
      targetFleetName ? availablePlaces(targetFleetName) : [],
    );
    const [startLocation, setStartLocation] = React.useState(
      listOfPlaces && listOfPlaces.length >= 2 ? listOfPlaces[0] : '',
    );
    const [finishLocation, setFinishLocation] = React.useState(
      listOfPlaces && listOfPlaces.length >= 2 ? listOfPlaces[1] : '',
    );

    React.useLayoutEffect(() => {
      if (listOfPlaces) {
        setStartLocation(listOfPlaces.length >= 2 ? listOfPlaces[0] : '');
        setFinishLocation(listOfPlaces.length >= 2 ? listOfPlaces[1] : '');
      }
    }, [listOfPlaces]);

    // Error states
    const [targetFleetNameError, setTargetFleetNameError] = React.useState('');
    const [numLoopsError, setNumLoopsError] = React.useState('');
    const [startLocationError, setStartLocationError] = React.useState('');
    const [finishLocationError, setFinishLocationError] = React.useState('');

    const cleanUpForm = () => {
      setTargetFleetName(targetFleetName);
      setNumLoops(0);
      setListOfPlaces(targetFleetName ? availablePlaces(targetFleetName) : []);
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

    const handleTargetFleetNameChange = (_: ChangeEvent<unknown>, value: string | null) => {
      const newFleetName = value || fleetNames[0];
      const newPlaces = availablePlaces(newFleetName) || [];
      setListOfPlaces(newPlaces);
      setStartLocation((cur) => {
        if (newPlaces.includes(cur)) {
          return cur;
        }
        return newPlaces.length >= 2 ? newPlaces[0] : '';
      });
      setFinishLocation((cur) => {
        if (newPlaces.includes(cur)) {
          return cur;
        }
        return newPlaces.length >= 2 ? newPlaces[1] : '';
      });
      setTargetFleetName(newFleetName);
    };

    const handleSubmit = (ev: React.FormEvent) => {
      ev.preventDefault();
      if (isFormValid()) {
        doLoopRequest && doLoopRequest(targetFleetName, numLoops, startLocation, finishLocation);
        cleanUpForm();
      }
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
          <TextField
            onChange={(e) => {
              setNumLoops(e.target.value ? parseInt(e.target.value) : 0);
            }}
            placeholder="Number of loops"
            type="number"
            value={numLoops || ''}
            className={commandFormsClasses.input}
            label="Number of loops"
            variant="outlined"
            error={!!numLoopsError}
            helperText={numLoopsError}
          />
        </div>

        <div className={commandFormsClasses.divForm}>
          <Autocomplete
            getOptionLabel={(option) => option}
            onChange={(_, value) => setStartLocation(value || '')}
            options={listOfPlaces ? listOfPlaces : []}
            renderInput={(params) => (
              <TextField
                {...params}
                label="Pick Start Location"
                placeholder="Pick Start Location"
                variant="outlined"
                error={!!startLocationError}
                helperText={startLocationError}
              />
            )}
            value={startLocation ? startLocation : null}
          />
        </div>

        <div className={commandFormsClasses.divForm}>
          <Autocomplete
            getOptionLabel={(option) => option}
            onChange={(_, value) => setFinishLocation(value || '')}
            options={listOfPlaces ? listOfPlaces : []}
            renderInput={(params) => (
              <TextField
                {...params}
                label="Pick Finish Location"
                placeholder="Pick Finish Location"
                variant="outlined"
                error={!!finishLocationError}
                helperText={finishLocationError}
              />
            )}
            value={finishLocation ? finishLocation : null}
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

export default LoopRequestForm;
