import { TextField, TextFieldProps } from '@material-ui/core';
import React from 'react';

export type PositiveIntField = Omit<TextFieldProps, 'type' | 'inputProps'>;

export function PositiveIntField(props: PositiveIntField): JSX.Element {
  return (
    <TextField
      {...props}
      type="number"
      inputProps={{ min: 0 }}
      onKeyDown={(ev) => {
        if ('-+.'.indexOf(ev.key) >= 0) {
          ev.preventDefault();
        }
      }}
    />
  );
}
