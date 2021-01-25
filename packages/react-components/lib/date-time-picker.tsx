import React from 'react';
import { createStyles, makeStyles, Theme } from '@material-ui/core/styles';
import {
  DateTimePickerProps,
  KeyboardDateTimePicker,
  MuiPickersUtilsProvider,
} from '@material-ui/pickers';
import MomentUtils from '@date-io/moment';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    container: {
      display: 'flex',
      flexWrap: 'wrap',
    },
    textField: {
      marginLeft: theme.spacing(1),
      marginRight: theme.spacing(1),
      width: 200,
    },
  }),
);

export default function DateAndTimePickers(props: DateTimePickerProps): React.ReactElement {
  const classes = useStyles();
  const { name, label, value, error, ...rest } = props;
  return (
    <MuiPickersUtilsProvider utils={MomentUtils}>
      <KeyboardDateTimePicker
        id={`${name}-datetime-local`}
        value={value ? value : new Date().toISOString().substr(0, 16)}
        label={label}
        format="yyyy/MM/DD HH:mm"
        inputVariant="outlined"
        variant="inline"
        ampm={false}
        {...rest}
      />
    </MuiPickersUtilsProvider>
  );
}
