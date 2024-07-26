import React from 'react';
import { FormControl, FormControlLabel, Radio, RadioGroup } from '@mui/material';

interface EventEditDeletePopupProps {
  currentValue: string;
  allValue: string;
  value: string;
  onChange: (event: React.ChangeEvent<HTMLInputElement>) => void;
}

export function EventEditDeletePopup({
  currentValue,
  allValue,
  value,
  onChange,
}: EventEditDeletePopupProps): JSX.Element {
  return (
    <FormControl fullWidth={true}>
      <RadioGroup
        aria-labelledby="task-schedule-event-edit-delete-popup-radio-buttons-group"
        name="task-schedule-event-edit-delete-popup-radio-buttons-group"
        value={value}
        onChange={onChange}
      >
        <FormControlLabel value={currentValue} control={<Radio />} label={'This event'} />
        <FormControlLabel
          value={allValue}
          control={<Radio />}
          label={'All events in this schedule'}
        />
      </RadioGroup>
    </FormControl>
  );
}
