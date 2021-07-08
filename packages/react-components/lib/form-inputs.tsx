import { TextField, TextFieldProps } from '@material-ui/core';
import React from 'react';

export interface PositiveIntField
  extends Omit<TextFieldProps, 'type' | 'value' | 'inputProps' | 'onChange'> {
  value: number;
  onChange?(ev: React.ChangeEvent, int: number): void;
}

export function PositiveIntField(props: PositiveIntField): JSX.Element {
  const [value, setValue] = React.useState(props.value.toString());

  React.useEffect(() => {
    setValue(props.value.toString());
  }, [props.value]);

  return (
    <TextField
      {...props}
      type="number"
      value={value}
      inputProps={{ min: 0 }}
      onKeyDown={(ev) => {
        if ('-+.'.indexOf(ev.key) >= 0) {
          ev.preventDefault();
        }
        props.onKeyDown && props.onKeyDown(ev);
      }}
      onChange={(ev) => {
        const int = parseInt(ev.target.value);
        if (int > 0) {
          props.onChange && props.onChange(ev, int);
        }
        setValue(ev.target.value);
      }}
    />
  );
}
