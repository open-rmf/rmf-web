import React from 'react';
import { TextField, Typography, Divider } from '@material-ui/core';

import { StyleTyping, FormProps } from './Utils';

const styles: StyleTyping = {
  root: {
    margin: '0 auto',
    width: '40%',
  },
  heading: {
    padding: '0.5rem',
  },
  textFieldWrapper: {
    padding: '1rem 0.5rem',
  },
  textField: {
    width: '100%',
  },
};

export default function TextFieldComponent(props: FormProps) {
  const { errorState, errorMessage, labels } = props;

  return (
    <div style={styles.root}>
      <div style={styles.heading}>
        <Typography variant="body1">
          Shown here is the text field that we use for manually entering values. The examples below
          shows how the text field appears when inputs are normal and errornous.
        </Typography>
      </div>
      {errorState.map((state, index) => {
        return (
          <React.Fragment>
            <Divider />
            <div style={styles.textFieldWrapper}>
              <TextField
                id="numLoops"
                name="numLoops"
                placeholder="Number of loops"
                type="number"
                label={labels[index]}
                variant="outlined"
                error={state}
                helperText={errorMessage[index]}
                style={styles.textField}
              />
            </div>
          </React.Fragment>
        );
      })}
    </div>
  );
}
