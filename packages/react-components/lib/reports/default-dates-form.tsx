import { DateTimePicker } from '@mui/x-date-pickers/DateTimePicker';
import { styled, TextField } from '@mui/material';
import Button from '@mui/material/Button';
import React from 'react';
import { LogQueryPayload } from '.';

interface DefaultDatesFormProps {
  search?: (payload: LogQueryPayload) => void;
  fromLogDate?: Date;
  toLogDate?: Date;
  onSelectFromDate?: (date: unknown) => void;
  onSelectToDate?: (date: unknown) => void;
}

const classes = {
  searchForm: 'traj-time-control-root',
  searchButton: 'traj-time-control-container',
};
const StyledDiv = styled('div')(() => ({
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
    <StyledDiv>
      <div className={classes.searchForm}>
        <DateTimePicker
          maxDate={new Date()}
          label="From"
          value={fromLogDate}
          onChange={onSelectFromDate}
          data-unix-time="asdjs"
          renderInput={(props) => (
            <TextField
              id="from-log-date-input"
              {...props}
              inputProps={{ ...props.inputProps, 'data-unix': fromLogDate?.valueOf() }}
            />
          )}
        />
        <DateTimePicker
          maxDate={new Date()}
          label="To"
          value={toLogDate}
          onChange={onSelectToDate}
          data-unix-time={toLogDate?.valueOf()}
          renderInput={(props) => (
            <TextField
              id="to-log-date-input"
              {...props}
              inputProps={{ ...props.inputProps, 'data-unix': toLogDate?.valueOf() }}
            />
          )}
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
    </StyledDiv>
  ) : null;
};
