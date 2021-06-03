import React from 'react';
import {
  DateTimePickerProps,
  KeyboardDateTimePicker,
  MuiPickersUtilsProvider,
} from '@material-ui/pickers';
import DateFnsUtils from '@date-io/date-fns';
import { format } from 'date-fns';

export default function DateAndTimePickers(props: DateTimePickerProps): React.ReactElement {
  const { name, label, value, ...rest } = props;
  return (
    <MuiPickersUtilsProvider utils={DateFnsUtils}>
      <KeyboardDateTimePicker
        id={`${name}-datetime-local`}
        value={value ? value : format(new Date(), 'MM/dd/yyyy HH:mm')}
        label={label}
        format="MM/dd/yyyy HH:mm"
        inputVariant="outlined"
        variant="inline"
        ampm={false}
        {...rest}
      />
    </MuiPickersUtilsProvider>
  );
}
