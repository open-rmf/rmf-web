import React from 'react';
import {
  FormControl,
  InputLabel,
  MenuItem,
  Select,
  SelectChangeEvent,
  styled,
} from '@mui/material';

const classes = {
  formControl: 'search-filter-formcontrol',
};
const StyledSearchFilter = styled('div')(({ theme }) => ({
  [`& .${classes.formControl}`]: {
    margin: theme.spacing(1),
    minWidth: '230px',
  },
}));

interface SearchFilterProps {
  options: { label: string; value: string }[];
  handleOnChange: (event: SelectChangeEvent<React.ReactText>, child: React.ReactNode) => void;
  label: string;
  name: string;
  currentValue: string | number;
}

export const SearchFilter = (props: SearchFilterProps): React.ReactElement => {
  const { handleOnChange, options, name, label, currentValue } = props;

  return (
    <StyledSearchFilter>
      <div>
        <FormControl variant="outlined" className={classes.formControl}>
          <InputLabel id={`${name}-select-outlined-label`}>{label}</InputLabel>
          <Select
            labelId={`${name}-select-outlined-label`}
            id={`${name}-select-outlined`}
            type="string"
            value={currentValue}
            onChange={handleOnChange}
            label={label}
          >
            {options.map((option) => {
              return (
                <MenuItem key={option.label} value={option.value}>
                  {option.label}
                </MenuItem>
              );
            })}
          </Select>
        </FormControl>
      </div>
    </StyledSearchFilter>
  );
};
