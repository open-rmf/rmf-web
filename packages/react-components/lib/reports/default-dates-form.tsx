import React from 'react';
import { makeStyles } from '@material-ui/core';
import Button from '@material-ui/core/Button';
import DateAndTimePickers from '../date-time-picker';

import moment from 'moment';
import { MaterialUiPickersDate } from '@material-ui/pickers/typings/date';
import { LogQueryPayload } from '.';

interface DefaultDatesFormProps {
  search?: (payload: LogQueryPayload) => void;
}

export const DefaultDatesForm = (props: DefaultDatesFormProps): React.ReactElement => {
  const { search } = props;

  const [fromLogDate, setFromLogDate] = React.useState<MaterialUiPickersDate>(moment(new Date()));
  const [toLogDate, setToLogDate] = React.useState<MaterialUiPickersDate>(moment(new Date()));

  const classes = useStyles();

  const searchQuery = () => {
    search && search({ toLogDate, fromLogDate });
  };

  const handleFromLogDateChange = React.useCallback((date: MaterialUiPickersDate) => {
    setFromLogDate(date);
  }, []);

  const handleToLogDateChange = React.useCallback((date: MaterialUiPickersDate) => {
    setToLogDate(date);
  }, []);

  return (
    <>
      <div className={classes.searchForm}>
        <DateAndTimePickers
          name="fromLogDate"
          maxDate={new Date()}
          label="From"
          value={fromLogDate}
          onChange={handleFromLogDateChange}
        />
        <DateAndTimePickers
          name="toLogDate"
          maxDate={new Date()}
          label="To"
          value={toLogDate}
          onChange={handleToLogDateChange}
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

const useStyles = makeStyles(() => ({
  searchForm: {
    display: 'flex',
    justifyContent: 'space-evenly',
  },
  searchButton: {
    width: '100%',
  },
}));
