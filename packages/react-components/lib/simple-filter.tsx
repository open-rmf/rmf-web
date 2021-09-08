import React from 'react';
import { TextField, Divider } from '@mui/material';
import { makeStyles } from '@mui/styles';

export interface OnFilterChangeEvent {
  name?: string | undefined;
  value: string;
}

export interface SimpleFilterProps {
  onChange?: (e: React.ChangeEvent<OnFilterChangeEvent>) => void;
  value: string;
}

const useStyles = makeStyles(() => ({
  simpleFilter: {
    margin: '1rem',
  },
  filterBar: {
    width: '100%',
  },
  divider: {
    margin: '1.5rem 0',
  },
}));

export const SimpleFilter = (props: SimpleFilterProps): JSX.Element => {
  const classes = useStyles();

  const { onChange, value } = props;

  return (
    <div className={classes.simpleFilter}>
      <TextField
        label="Filter"
        value={value}
        variant="outlined"
        onChange={onChange}
        className={classes.filterBar}
        aria-label="text-input"
        data-component="simple-filter"
      />
      <Divider className={classes.divider} />
    </div>
  );
};
