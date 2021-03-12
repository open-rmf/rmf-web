import React from 'react';
import { TextField, makeStyles } from '@material-ui/core';

export interface SimpleSearchProps {
  onChange?: (e: any) => void;
  disabled: boolean;
  value: string;
}

const useStyles = makeStyles(() => ({
  simpleSearch: {
    margin: '1rem',
    display: 'flex',
    justifyContent: 'center',
  },
}));

export const SimpleSearch = (props: SimpleSearchProps): JSX.Element => {
  const classes = useStyles();

  const { onChange, disabled, value } = props;

  return (
    <TextField
      label="Search"
      value={value}
      disabled={disabled}
      className={classes.simpleSearch}
      variant="outlined"
      onChange={onChange}
    />
  );
};
