import React, { ChangeEvent } from 'react';
import {
  FormControl,
  FormControlLabel,
  FormLabel,
  Grid,
  makeStyles,
  Radio,
  RadioGroup,
} from '@material-ui/core';

const useStyles = makeStyles(() => ({
  root: {
    flexWrap: 'nowrap',
  },
}));
interface RadioGroupProps {
  formLabel: string;
  options: string[];
  radioGroupName: string;
  row?: boolean;
  onHandleChange: (value: string) => void;
}

export const RadioButtonGroup = (props: RadioGroupProps): JSX.Element => {
  const { formLabel, radioGroupName, options, row, onHandleChange } = props;
  const [radioOptions, setRadioOptions] = React.useState<string[]>([]);
  const [value, setValue] = React.useState('');
  const classes = useStyles();

  React.useEffect(() => {
    setRadioOptions(options);
  }, [options]);

  const handleChange = (event: ChangeEvent<HTMLInputElement>) => {
    const elem = event.target as HTMLInputElement;
    setValue(elem.value);
    onHandleChange(elem.value);
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
              classes={{ root: classes.root }}
            >
              {populateRadioButtons(radioOptions)}
            </RadioGroup>
          </FormControl>
        </Grid>
      </Grid>
    </div>
  );
};
