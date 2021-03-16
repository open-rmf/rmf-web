import React from 'react';
import { TextField, makeStyles, Divider } from '@material-ui/core';

export interface OnChangeEvent {
  name?: string | undefined;
  value: string;
}

export interface SimpleSearchProps {
  onChange?: (e: React.ChangeEvent<OnChangeEvent>) => void;
  value: string;
}

const useStyles = makeStyles((theme) => ({
  simpleSearch: {
    margin: '1rem',
  },
  clearSearch: {
    color: theme.palette.error.main,
  },
  searchBar: {
    width: '100%',
  },
  divider: {
    margin: '1.5rem 0',
  },
}));

export const SimpleSearch = (props: SimpleSearchProps): JSX.Element => {
  const classes = useStyles();

  const { onChange, value } = props;

  return (
    <div className={classes.simpleSearch}>
      <TextField
        label="Search"
        value={value}
        variant="outlined"
        onChange={onChange}
        className={classes.searchBar}
        aria-label="text-input"
      />
      <Divider className={classes.divider} />
    </div>
  );
};
