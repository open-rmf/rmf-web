import React from 'react';
import SearchIcon from '@material-ui/icons/Search';
import {
  FormControl,
  IconButton,
  InputLabel,
  makeStyles,
  MenuItem,
  Select,
  TextField,
} from '@material-ui/core';
import Button from '@material-ui/core/Button';

interface SearchFilterProps {
  handleOnChange: () => void;
}

export const SearchFilter = (props: SearchFilterProps) => {
  const { handleOnChange } = props;
  const [searchText, setSearchText] = React.useState('');
  const [error, setError] = React.useState('');
  const classes = useStyles();

  const searchQuery = () => {
    if (!searchText) {
      setError('Cannot be empty');
      return;
    }
    // search();
  };

  return (
    <>
      <div>
        <FormControl variant="outlined" className={classes.formControl}>
          <InputLabel id="demo-simple-select-outlined-label">Age</InputLabel>
          <Select
            labelId="demo-simple-select-outlined-label"
            id="demo-simple-select-outlined"
            type="string"
            value={searchText}
            onChange={(e) => {
              setSearchText('');
            }}
            label="Age"
          >
            <MenuItem value="">
              <em>None</em>
            </MenuItem>
            <MenuItem value={10}>Ten</MenuItem>
            <MenuItem value={20}>Twenty</MenuItem>
            <MenuItem value={30}>Thirty</MenuItem>
          </Select>
        </FormControl>
      </div>
    </>
  );
};

const useStyles = makeStyles((theme) => ({
  formControl: {
    margin: theme.spacing(1),
    minWidth: 120,
  },
}));
