import React from 'react';
import { TextField, styled } from '@mui/material';
import Button from '@mui/material/Button';
import DateAndTimePickers from '../date-time-picker';

import { LogQueryPayload } from '.';

interface DefaultDatesFormProps {
  search?: (payload: LogQueryPayload) => void;
  fromLogDate?: Date;
  toLogDate?: Date;
  onSelectFromDate?: (date: any) => void;
  onSelectToDate?: (date: any) => void;
}

const classes = {
  searchForm: 'traj-time-control-root',
  searchButton: 'traj-time-control-container',
};
const DefaultDatesFormRoot = styled('div')(() => ({
  [`& .${classes.searchForm}`]: {
    display: 'flex',
    justifyContent: 'space-evenly',
  },
  [`& .${classes.searchButton}`]: {
    width: '100%',
  },
}));

export const DefaultDatesForm = (props: DefaultDatesFormProps): JSX.Element | null => {
  const { search, fromLogDate, toLogDate, onSelectToDate, onSelectFromDate } = props;
  const searchQuery = () => {
    search && search({ toLogDate, fromLogDate });
  };

  return onSelectFromDate && onSelectToDate ? (
    <DefaultDatesFormRoot>
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
    </DefaultDatesFormRoot>
  ) : null;
};
