import React from 'react';
import { FormControl, makeStyles, TextField } from '@material-ui/core';
import Button from '@material-ui/core/Button';
import { SearchFilter } from './search-filter';
import DateAndTimePickers from '../date-time-picker';
import { LogLevel } from './log-level';

interface SearchLogFormProps {
  logLabelValues: { label: string; value: string }[];
  search?: (searchText: string, logLabel: string, logLevel: string, rowsCount: number) => void;
}

const logLevelValues = [
  { label: 'FATAL', value: LogLevel.Fatal },
  { label: 'ERROR', value: LogLevel.Error },
  { label: 'WARN', value: LogLevel.Warn },
  { label: 'INFO', value: LogLevel.Info },
  { label: 'DEBUG', value: LogLevel.Debug },
];

export const SearchLogForm = (props: SearchLogFormProps): React.ReactElement => {
  const { search, logLabelValues } = props;
  const [searchText, setSearchText] = React.useState('');
  // The log contains information from different services, the label help us differentiate the service
  const [logLabel, setLogLabel] = React.useState('');
  const [logLevel, setLogLevel] = React.useState(LogLevel.Error);
  const [rowsCount, setRowsCount] = React.useState(100);
  const [fromLogDate, setFromLogDate] = React.useState(new Date().toISOString().substr(0, 16));
  const [toLogDate, setToLogDate] = React.useState(new Date().toISOString().substr(0, 16));
  const [fromLogDateError, setFromLogDateError] = React.useState('');
  const [toLogDateError, setToLogDateError] = React.useState('');

  const classes = useStyles();

  const isFormValid = () => {
    let isValid = true;
    if (!fromLogDate) {
      setFromLogDateError('Cannot be empty');
      isValid = false;
    }
    if (!toLogDate) {
      setToLogDateError('Cannot be empty');
      isValid = false;
    }
    return isValid;
  };

  const searchQuery = () => {
    if (!isFormValid()) return;
    console.log(searchText, logLabel, logLevel, rowsCount, toLogDate, fromLogDate);
    search && search(searchText, logLabel, logLevel, rowsCount);
  };

  const rowsCountValues = [
    { label: '50', value: '50' },
    { label: '100', value: '100' },
    { label: '200', value: '200' },
    { label: '500', value: '500' },
  ];

  const handleLogLabelChange = React.useCallback(
    (event: React.ChangeEvent<{ name?: string; value: unknown }>) => {
      setLogLabel(event.target.value as string);
    },
    [],
  );

  const handleLogLevelChange = React.useCallback(
    (event: React.ChangeEvent<{ name?: string; value: unknown }>) => {
      setLogLevel(event.target.value as LogLevel);
    },
    [],
  );

  const handleRowsNumberChange = React.useCallback(
    (event: React.ChangeEvent<{ name?: string; value: unknown }>) => {
      setRowsCount(event.target.value as number);
    },
    [],
  );

  const handleFromLogDateChange = React.useCallback(
    (event: React.ChangeEvent<{ name?: string; value: unknown }>) => {
      setFromLogDate(event.target.value as string);
    },
    [],
  );

  const handleToLogDateChange = React.useCallback(
    (event: React.ChangeEvent<{ name?: string; value: unknown }>) => {
      setToLogDate(event.target.value as string);
    },
    [],
  );

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
          />
        </FormControl>

        <SearchFilter
          options={logLabelValues}
          name="log-picker"
          label="Pick Log Label"
          aria-label="log-label-picker"
          handleOnChange={handleLogLabelChange}
          currentValue={logLabel}
        ></SearchFilter>

        <SearchFilter
          options={logLevelValues}
          name="log-level"
          label="Pick Log Level"
          handleOnChange={handleLogLevelChange}
          currentValue={logLevel}
        ></SearchFilter>

        <DateAndTimePickers
          name="fromLogDate"
          label="From"
          date={fromLogDate}
          error={fromLogDateError}
          handleDateChange={handleFromLogDateChange}
        ></DateAndTimePickers>

        <DateAndTimePickers
          name="toLogDate"
          label="To"
          error={toLogDateError}
          date={toLogDate}
          handleDateChange={handleToLogDateChange}
        ></DateAndTimePickers>

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
        onClick={searchQuery}
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
