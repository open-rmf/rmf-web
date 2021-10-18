import React from 'react';
import { makeStyles } from '@material-ui/core';
import Button from '@material-ui/core/Button';
import DateAndTimePickers from '../date-time-picker';

import { MaterialUiPickersDate } from '@material-ui/pickers/typings/date';
import { LogQueryPayload } from '.';

interface DefaultDatesFormProps {
  search?: (payload: LogQueryPayload) => void;
  fromLogDate?: MaterialUiPickersDate;
  toLogDate?: MaterialUiPickersDate;
  onSelectFromDate?: (date: MaterialUiPickersDate) => void;
  onSelectToDate?: (date: MaterialUiPickersDate) => void;
}

export const DefaultDatesForm = (props: DefaultDatesFormProps): JSX.Element | null => {
  const { search, fromLogDate, toLogDate, onSelectToDate, onSelectFromDate } = props;

  const classes = useStyles();

  const searchQuery = () => {
    search && search({ toLogDate, fromLogDate });
  };

  return onSelectFromDate && onSelectToDate ? (
    <>
      <div className={classes.searchForm}>
        <DateAndTimePickers
          name="fromLogDate"
          maxDate={new Date()}
          label="From"
          value={fromLogDate}
          onChange={onSelectFromDate}
        />
        <DateAndTimePickers
          name="toLogDate"
          maxDate={new Date()}
          label="To"
          value={toLogDate}
          onChange={onSelectToDate}
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
  ) : null;
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
