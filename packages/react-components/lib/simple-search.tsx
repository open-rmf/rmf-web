import React from 'react';
import { TextField, makeStyles, IconButton } from '@material-ui/core';
import CloseIcon from '@material-ui/icons/Close';

export interface SimpleSearchProps {
  onChange?: (e: any) => void;
  disabled: boolean;
  value: string;
}

const useStyles = makeStyles((theme) => ({
  simpleSearch: {
    margin: '1rem',
    display: 'flex',
    justifyContent: 'space-between',
  },
  clearSearch: {
    color: theme.palette.error.main,
  },
  searchBar: {
    width: '100%',
  },
}));

export const SimpleSearch = (props: SimpleSearchProps): JSX.Element => {
  const classes = useStyles();

  const { onChange, disabled, value } = props;

  return (
    <div className={classes.simpleSearch}>
      <TextField
        label="Search"
        value={value}
        disabled={disabled}
        variant="outlined"
        onChange={onChange}
        className={classes.searchBar}
      />
      <IconButton className={classes.clearSearch}>
        <CloseIcon />
      </IconButton>
    </div>
  );
};
