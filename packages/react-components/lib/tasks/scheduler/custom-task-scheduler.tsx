import { Fab, Grid, makeStyles, MenuItem, TextField, Typography } from '@material-ui/core';
import React from 'react';
import { RecurrentRules, WeekDay } from '.';
import { PositiveIntField } from '../..';
import { ReducerTaskDispatch, TaskState } from '../task-reducer';

const useStyles = makeStyles((theme) => ({
  taskList: {
    flex: '1 1 auto',
  },
  selectedTask: {
    background: theme.palette.action.focus,
  },
}));

interface CustomTaskScheduleProps {
  state: TaskState;
  dispatch: ReducerTaskDispatch;
  selectedDate?: Date;
}

interface DaysButtonProps {
  text: string;
  onClick?: () => void;
}

export const DaysButton = (props: DaysButtonProps): JSX.Element => {
  const { text, onClick } = props;
  const [selected, setSelected] = React.useState(false);

  const handleClick = () => {
    if (onClick) {
      onClick();
    }
    setSelected(!selected);
  };

  return (
    <Fab color={selected ? 'primary' : undefined} onClick={handleClick}>
      {text}
    </Fab>
  );
};

export const CustomTaskSchedule = (props: CustomTaskScheduleProps): JSX.Element => {
  const { state, dispatch } = props;
  const classes = useStyles();

  const handleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    dispatch.setFrequencyTypeCustom(event.target.value as string);
  };

  const handleDaysClick = (day: number) => {
    if (state.daysOfWeek.includes(day)) {
      const filteredWeekDays = state.daysOfWeek.filter((e) => e !== day);
      dispatch.setDaysOfWeek(filteredWeekDays);
      return;
    }

    const newArray = state.daysOfWeek.concat(day);
    dispatch.setDaysOfWeek(newArray);
  };

  const customFrequencyTypeList = React.useMemo(() => {
    if (state.daysOfWeek.length === 0) {
      return RecurrentRules.getBasicRecurrenceTypeList();
    } else {
      return RecurrentRules.getDaysOfWeekRecurrenceTypeList();
    }
  }, [state.daysOfWeek]);
  return (
    <>
      <Grid
        container
        wrap="nowrap"
        style={{
          alignItems: 'center',
          justifyContent: 'center',
        }}
      >
        <Grid
          style={{
            flex: '0 1 5em',
          }}
        >
          <Typography>Repeat Every</Typography>
        </Grid>
        <Grid
          style={{
            flex: '0.5 1 1rem',
          }}
        >
          <PositiveIntField
            id="frequency"
            label="Frequency"
            margin="normal"
            variant="outlined"
            value={state.frequency}
            onChange={(_ev, val) => {
              dispatch.setFrequency(val || 0);
            }}
          />
        </Grid>
        <Grid style={{ flexGrow: 1 }}>
          <TextField
            select
            id="frequencyType"
            label="Type"
            variant="outlined"
            fullWidth
            margin="normal"
            value={state.frequencyTypeCustom}
            onChange={handleChange}
          >
            {customFrequencyTypeList.map((option) => (
              <MenuItem key={option.key} value={option.key as string}>
                {option.value}
              </MenuItem>
            ))}
          </TextField>
        </Grid>
      </Grid>
      <br />
      <div
        style={{
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
        }}
      >
        <Typography>Repeat On</Typography>
        <DaysButton text={'S'} onClick={() => handleDaysClick(WeekDay.Sunday)}></DaysButton>
        <DaysButton text={'M'} onClick={() => handleDaysClick(WeekDay.Monday)}></DaysButton>
        <DaysButton text={'T'} onClick={() => handleDaysClick(WeekDay.Tuesday)}></DaysButton>
        <DaysButton text={'W'} onClick={() => handleDaysClick(WeekDay.Wednesday)}></DaysButton>
        <DaysButton text={'T'} onClick={() => handleDaysClick(WeekDay.Thursday)}></DaysButton>
        <DaysButton text={'F'} onClick={() => handleDaysClick(WeekDay.Friday)}></DaysButton>
        <DaysButton text={'S'} onClick={() => handleDaysClick(WeekDay.Saturday)}></DaysButton>
      </div>
    </>
  );
};

export default CustomTaskSchedule;
