import React from 'react';
import { TextField, Divider, styled } from '@material-ui/core';

export interface OnFilterChangeEvent {
  name?: string | undefined;
  value: string;
}

export interface SimpleFilterProps {
  onChange?: (e: React.ChangeEvent<OnFilterChangeEvent>) => void;
  value: string;
}

const classes = {
  simpleFilter: 'simple-filter-root',
  filterBar: 'simple-filter-filterbar',
  divider: 'simple-filter-divider',
};
const SimpleFilterRoot = styled('div')(() => ({
  [`&.${classes.simpleFilter}`]: {
    margin: '1rem',
  },
  [`& .${classes.filterBar}`]: {
    width: '100%',
  },
  [`& .${classes.divider}`]: {
    margin: '1.5rem 0',
  },
}));

export const SimpleFilter = (props: SimpleFilterProps): JSX.Element => {
  const { onChange, value } = props;

  return (
    <SimpleFilterRoot className={classes.simpleFilter}>
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
    </SimpleFilterRoot>
  );
};
