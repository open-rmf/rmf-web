import React from 'react';
import { TextField, SelectChangeEvent, styled } from '@mui/material';
import Button from '@mui/material/Button';
import { SearchFilter } from './search-filter';
import DateAndTimePickers from '../../date-time-picker';
import { DateTimePickerProps } from '@mui/lab';
import { LogLevel } from './log-level';

import { LogQueryPayload } from '.';

interface SearchLogFormProps {
  logLabelValues: { label: string; value: string }[];
  search?: (payload: LogQueryPayload) => void;
}

const classes = {
  searchForm: 'search-log-search-from',
  searchButton: 'search-log-search-button',
  formControl: 'search-log-form-form-control',
  background: 'search-log-form-background',
};
const SearchLogRoot = styled('div')(({ theme }) => ({
  [`&.${classes.background}`]: {
    backgroundColor: theme.palette.background.paper,
  },
  [`& .${classes.searchForm}`]: {
    display: 'grid',
    gridTemplateColumns: '1fr 1fr 1fr 1fr',
    alignItems: 'center',
    justifyItems: 'center',
  },
  [`& .${classes.searchButton}`]: {
    width: '100%',
  },
  [`& .${classes.formControl}`]: {
    margin: theme.spacing(1),
    minWidth: 120,
  },
}));

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
  const [fromLogDate, setFromLogDate] = React.useState<unknown>(new Date());
  const [toLogDate, setToLogDate] = React.useState<unknown>(new Date());

  const searchQuery = () => {
    search && search({ toLogDate, fromLogDate, logLabel, logLevel });
  };

  const handleLogLabelChange = React.useCallback((event: SelectChangeEvent<React.ReactText>) => {
    setLogLabel(event.target.value as string);
  }, []);

  const handleLogLevelChange = React.useCallback((event: SelectChangeEvent<React.ReactText>) => {
    setLogLevel(event.target.value as LogLevel);
  }, []);

  const handleFromLogDateChange: Required<DateTimePickerProps>['onChange'] = React.useCallback(
    (date) => {
      setFromLogDate(date);
    },
    [],
  );

  const handleToLogDateChange: Required<DateTimePickerProps>['onChange'] = React.useCallback(
    (date) => {
      setToLogDate(date);
    },
    [],
  );

  return (
    <SearchLogRoot className={classes.background}>
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
          renderInput={(props) => <TextField id={'From'} {...props} />}
        />
        <DateAndTimePickers
          maxDate={new Date()}
          label="To"
          value={toLogDate}
          onChange={handleToLogDateChange}
          renderInput={(props) => <TextField id={'To'} {...props} />}
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
    </SearchLogRoot>
  );
};
