import React from 'react';
import { FormControl, FormControlLabel, Radio, RadioGroup } from '@mui/material';

interface EventEditDeletePopupProps {
  currentValue: string;
  allValue: string;
  value: string;
  deleting: boolean;
  onChange: (event: React.ChangeEvent<HTMLInputElement>) => void;
}

export function EventEditDeletePopup({
  currentValue,
  allValue,
  value,
  deleting,
  onChange,
}: EventEditDeletePopupProps): JSX.Element {
  return (
    <FormControl fullWidth={true}>
      <RadioGroup
        aria-labelledby="schedule-delete-options-radio-buttons-group"
        name="schedule-delete-options-radio-buttons-group"
        value={value}
        onChange={onChange}
      >
        <FormControlLabel
          value={currentValue}
          control={<Radio />}
          label={`${deleting ? 'This event' : 'Edit this instance'}`}
        />
        <FormControlLabel
          value={allValue}
          control={<Radio />}
          label={`${deleting ? 'All events' : 'Edit series'}`}
        />
      </RadioGroup>
    </FormControl>
  );
}
