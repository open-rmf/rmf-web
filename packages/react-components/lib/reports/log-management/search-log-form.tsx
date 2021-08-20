import React from 'react';
import { makeStyles, TextField } from '@material-ui/core';
import Button from '@material-ui/core/Button';
import { SearchFilter } from './search-filter';
import DateAndTimePickers from '../../date-time-picker';
import { LogLevel } from './log-level';

import { LogQueryPayload } from '.';

interface SearchLogFormProps {
  logLabelValues: { label: string; value: string }[];
  search?: (payload: LogQueryPayload) => void;
}

const logLevelValues = [
  { label: 'ALL', value: LogLevel.All },
  { label: 'FATAL', value: LogLevel.Fatal },
  { label: 'ERROR', value: LogLevel.Error },
  { label: 'WARN', value: LogLevel.Warn },
  { label: 'INFO', value: LogLevel.Info },
  { label: 'DEBUG', value: LogLevel.Debug },
];

export const SearchLogForm = (props: SearchLogFormProps): React.ReactElement => {
  const { search, logLabelValues } = props;
  // The log contains information from different services, the label help us differentiate the service
  const [logLabel, setLogLabel] = React.useState('');
  const [logLevel, setLogLevel] = React.useState(LogLevel.All);
  const [fromLogDate, setFromLogDate] = React.useState<Date>(new Date());
  const [toLogDate, setToLogDate] = React.useState<Date>(new Date());

  const classes = useStyles();

  const searchQuery = () => {
    search && search({ toLogDate, fromLogDate, logLabel, logLevel });
  };

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

  const handleFromLogDateChange = React.useCallback((date: any) => {
    setFromLogDate(date);
  }, []);

  const handleToLogDateChange = React.useCallback((date: any) => {
    setToLogDate(date);
  }, []);

  return (
    <>
      <div className={classes.searchForm}>
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
        />

        <DateAndTimePickers
          maxDate={new Date()}
          label="From"
          value={fromLogDate}
          onChange={handleFromLogDateChange}
          renderInput={(props) => <TextField {...props} />}
        />
        <DateAndTimePickers
          maxDate={new Date()}
          label="To"
          value={toLogDate}
          onChange={handleToLogDateChange}
          renderInput={(props) => <TextField {...props} />}
        />
      </div>

      <br></br>
      <Button
        color="primary"
        variant="contained"
        className={classes.searchButton}
        onClick={searchQuery}
      >
        Retrieve Logs
      </Button>
    </>
  );
};

const useStyles = makeStyles((theme) => ({
  searchForm: {
    display: 'grid',
    gridTemplateColumns: '1fr 1fr 1fr 1fr',
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
