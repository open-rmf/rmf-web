import {
  Fab,
  FormControlLabel,
  Grid,
  makeStyles,
  MenuItem,
  Radio,
  RadioGroup,
  TextField,
  Typography,
} from '@material-ui/core';
import React from 'react';
import { RecurrentRules, RecurrenceType, WeekDay } from '.';
import { PositiveIntField } from '../..';
import { IconButton } from '@material-ui/core';
import FiberManualRecordIcon from '@material-ui/icons/FiberManualRecord';
import { KeyboardDateTimePicker } from '@material-ui/pickers';
import { ReducerTaskDispatch, TaskState } from '../task-reducer';
/**
 * Date hour to hout date timezone
 * Does not repeat | Daily | Weekly on Monday | Monthly on the first Monday | Anually on Month X | Every week day |custom
 * Custom recurrence. Repeat every [number] [week] [day] [month] [year]. Ends On date, after x recurrences.
 */

const useStyles = makeStyles((theme) => ({
  taskList: {
    flex: '1 1 auto',
    // minHeight: 400,
    // maxHeight: '50vh',
    // overflow: 'auto',
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
    if (state.dayOfWeek.includes(day)) {
      const filteredWeekDays = state.dayOfWeek.filter((e) => e !== day);
      dispatch.setDayOfWeek(filteredWeekDays);
      return;
    }

    state.dayOfWeek.push(day);
    dispatch.setDayOfWeek(state.dayOfWeek);
  };

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
            {RecurrentRules.getBasicRecurrenceTypeList().map((option) => (
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
      <br />
      <div>
        <KeyboardDateTimePicker
          id="start-time"
          value={state.endDatetime || new Date()}
          onChange={(date) => {
            if (!date) {
              return;
            }
            dispatch.setEndDatetime(date.toString());
          }}
          label="Ends"
          margin="normal"
          fullWidth
        />
      </div>
    </>
  );
};

interface SchedulerProps {
  state: TaskState;
  dispatch: ReducerTaskDispatch;
  selectedDate?: Date;
}

export const Scheduler = (props: SchedulerProps): JSX.Element => {
  const { selectedDate, state, dispatch } = props;
  const optionList = RecurrentRules.getRecurrenceTypeList(selectedDate);
  const handleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    dispatch.setFrequencyType(event.target.value as string);
  };

  return (
    <>
      <TextField
        select
        id="task-scheduler"
        label="Schedule a task"
        variant="outlined"
        fullWidth
        margin="normal"
        value={state.frequencyType || RecurrenceType.DoesNotRepeat}
        onChange={handleChange}
      >
        {optionList.map((option) => (
          <MenuItem key={option.key} value={option.key as string}>
            {option.value}
          </MenuItem>
        ))}
      </TextField>
      <br />
      {state.frequencyType === RecurrenceType.Custom && (
        <CustomTaskSchedule state={state} dispatch={dispatch} />
      )}
    </>
  );
};

export default Scheduler;
