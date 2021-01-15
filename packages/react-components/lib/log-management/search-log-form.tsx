import React from 'react';
import { FormControl, makeStyles, TextField } from '@material-ui/core';
import Button from '@material-ui/core/Button';
import { SearchFilter } from './search-filter';
import DateAndTimePickers from '../date-time-picker';

interface SearchLogFormProps {
  search?: () => void;
}

export const SearchLogForm = (props: SearchLogFormProps) => {
  const { search } = props;
  const [searchText, setSearchText] = React.useState('');
  const [sourceLog, setSourceLog] = React.useState('');
  const [logLevel, setLogLevel] = React.useState('');
  const [rowsCount, setRowsCount] = React.useState(100);
  const [error, setError] = React.useState('');
  const classes = useStyles();

  const searchQuery = () => {
    if (!searchText) {
      setError('Cannot be empty');
      return;
    }
    search && search();
  };

  const sourceLogValues = [
    { label: 'log1', value: 'log1' },
    { label: 'log2', value: 'log2' },
  ];

  const logLevelValues = [{ label: 'INFO', value: 'info' }];

  const rowsCountValues = [
    { label: '50', value: '50' },
    { label: '100', value: '100' },
  ];

  const handleSourceLogChange = (event: React.ChangeEvent<{ name?: string; value: unknown }>) => {
    setSourceLog(event.target.value as string);
  };

  const handleLogLevelChange = (event: React.ChangeEvent<{ name?: string; value: unknown }>) => {
    setLogLevel(event.target.value as string);
  };

  const handleRowsNumberChange = (event: React.ChangeEvent<{ name?: string; value: unknown }>) => {
    setRowsCount(event.target.value as number);
  };

  return (
    <>
      <div className={classes.searchForm}>
        <FormControl variant="outlined" className={classes.formControl}>
          <TextField
            onChange={(e) => {
              setSearchText(e.target.value);
            }}
            placeholder={'Log contains...'}
            type="string"
            value={searchText}
            variant="outlined"
            error={!!error}
            helperText={error}
          />
        </FormControl>
        <SearchFilter
          options={sourceLogValues}
          name="log-picker"
          label="Pick Log File"
          handleOnChange={handleSourceLogChange}
          currentValue={sourceLog}
        ></SearchFilter>
        <SearchFilter
          options={logLevelValues}
          name="log-level"
          label="Pick Log Level"
          handleOnChange={handleLogLevelChange}
          currentValue={logLevel}
        ></SearchFilter>
        <DateAndTimePickers name="fromLogDate" label="From"></DateAndTimePickers>
        <DateAndTimePickers name="toLogDate" label="To"></DateAndTimePickers>
        <SearchFilter
          options={rowsCountValues}
          name="rowsCount"
          label="Rows number"
          handleOnChange={handleRowsNumberChange}
          currentValue={rowsCount}
        ></SearchFilter>
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
  searchForm: {
    display: 'grid',
    gridTemplateColumns: 'repeat(3, 1fr)',
    alignItems: 'center',
    justifyItems: 'center',
  },
  searchButton: {
    width: '100%',
  },
  formControl: {
    margin: theme.spacing(1),
    minWidth: 120,
  },
}));
