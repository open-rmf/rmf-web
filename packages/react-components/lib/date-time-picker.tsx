import React from 'react';
import { DateTimePickerProps, DateTimePicker, LocalizationProvider } from '@material-ui/pickers';
import DateFnsAdapter from '@material-ui/pickers/adapter/date-fns';
import { format } from 'date-fns';

export default function DateAndTimePickers(props: DateTimePickerProps): React.ReactElement {
  const { label, value, ...rest } = props;
  return (
    <LocalizationProvider dateAdapter={DateFnsAdapter}>
      <DateTimePicker
        inputFormat={'MM/dd/yyyy HH:mm'}
        value={value ? value : format(new Date(), 'MM/dd/yyyy HH:mm')}
        label={label}
        {...rest}
      />
    </LocalizationProvider>
  );
}
