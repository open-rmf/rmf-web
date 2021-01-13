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
import { SearchFilter } from './search-filter';
import DateAndTimePickers from '../date-time-picker';

interface SearchLogFormProps {
  search?: () => void;
}

// const searchQuery = () => {
//   // TODO: We should add a debounce if we want to search on every click
//   axios({
//     method: 'post',
//     url: 'localhost:3030/',
//     headers: {
//       'Content-Type': 'application/json',
//     },
//     data: { query: search },
//   });
// };

export const SearchLogForm = (props: SearchLogFormProps) => {
  const { search } = props;
  const [searchText, setSearchText] = React.useState('');
  const [error, setError] = React.useState('');
  const classes = useStyles();

  const searchQuery = () => {
    if (!searchText) {
      setError('Cannot be empty');
      return;
    }
    search && search();
  };

  return (
    <>
      <div className={classes.formDiv}>
        <FormControl variant="outlined" className={classes.formControl}>
          <TextField
            onChange={(e) => {
              setSearchText(e.target.value);
            }}
            placeholder={'Log contains...'}
            type="string"
            value={searchText}
            variant="outlined"
            // className={classes.search}
            error={!!error}
            helperText={error}
            // InputProps={{
            //   classes: {
            //     input: classes.resize,
            //   },
            // }}
          />
        </FormControl>
        <SearchFilter></SearchFilter>
        <SearchFilter></SearchFilter>
        <DateAndTimePickers></DateAndTimePickers>
        <DateAndTimePickers></DateAndTimePickers>
        <SearchFilter></SearchFilter>
      </div>
      <br></br>
      <Button
        color="primary"
        variant="contained"
        className={classes.searchButton}
        onClick={() => searchQuery()}
      >
        Search
      </Button>
    </>
  );
};

const useStyles = makeStyles((theme) => ({
  searchDiv: {
    width: '100%',
  },
  formDiv: {
    display: 'flex',
    flexDirection: 'row',
  },
  search: {
    width: '70%',
  },
  searchButton: {
    width: '100%',
  },
  resize: {
    fontSize: '3rem',
    borderRadius: '0.5rem',
    backgroundColor: 'white',
  },
  formControl: {
    margin: theme.spacing(1),
    minWidth: 120,
  },
}));
