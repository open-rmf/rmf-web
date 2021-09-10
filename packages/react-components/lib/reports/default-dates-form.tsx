import React from 'react';
import { TextField } from '@material-ui/core';
import { makeStyles } from '@material-ui/styles';
import Button from '@material-ui/core/Button';
import DateAndTimePickers from '../date-time-picker';

import { LogQueryPayload } from '.';

interface DefaultDatesFormProps {
  search?: (payload: LogQueryPayload) => void;
  fromLogDate?: Date;
  toLogDate?: Date;
  onSelectFromDate?: (date: any) => void;
  onSelectToDate?: (date: any) => void;
}

export const DefaultDatesForm = (props: DefaultDatesFormProps) => {
  const { search, fromLogDate, toLogDate, onSelectToDate, onSelectFromDate } = props;

  const classes = useStyles();

  const searchQuery = () => {
    search && search({ toLogDate, fromLogDate });
  };

  return onSelectFromDate && onSelectToDate ? (
    <>
      <div className={classes.searchForm}>
        <DateAndTimePickers
          maxDate={new Date()}
          label="From"
          value={fromLogDate}
          onChange={onSelectFromDate}
          renderInput={(props) => <TextField {...props} />}
        />
        <DateAndTimePickers
          maxDate={new Date()}
          label="To"
          value={toLogDate}
          onChange={onSelectToDate}
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
