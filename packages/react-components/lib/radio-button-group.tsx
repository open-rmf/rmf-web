import React, { ChangeEvent } from 'react';
import {
  FormControl,
  FormControlLabel,
  FormLabel,
  Grid,
  Radio,
  RadioGroup,
} from '@material-ui/core';

interface RadioGroupProps {
  formLabel: string;
  radioGroupName: string;
  options: string[];
  row?: boolean;
  onHandleChange: () => void;
}

export const RadioButtonGroup = (props: RadioGroupProps): JSX.Element => {
  const { formLabel, radioGroupName, options, row, onHandleChange } = props;
  const [radioOptions, setRadioOptions] = React.useState<string[]>([]);
  const [value, setValue] = React.useState('');

  React.useEffect(() => {
    if (options.length > 0) {
      setRadioOptions(options);
      setValue(options[0]);
    }
  }, [options]);

  const handleChange = (event: ChangeEvent<HTMLInputElement>) => {
    const elem = event.target as HTMLInputElement;
    setValue(elem.value);
    onHandleChange();
  };

  const populateRadioButtons = (options: string[]): JSX.Element[] => {
    const radioButtons = options.map((option, index) => {
      return (
        <FormControlLabel key={option + index} value={option} control={<Radio />} label={option} />
      );
    });
    return radioButtons;
  };

  return (
    <div>
      <Grid container>
        <Grid item xs={12}>
          <FormControl component="fieldset">
            <FormLabel component="legend">{formLabel}</FormLabel>
            <RadioGroup
              row={row}
              aria-label={radioGroupName}
              name={radioGroupName}
              value={value}
              onChange={handleChange}
            >
              {populateRadioButtons(radioOptions)}
            </RadioGroup>
          </FormControl>
        </Grid>
      </Grid>
    </div>
  );
};
