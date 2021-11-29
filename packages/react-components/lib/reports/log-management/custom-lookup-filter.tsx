import {
  Checkbox,
  FormControl,
  Input,
  InputLabel,
  ListItemText,
  MenuItem,
  Select,
} from '@mui/material';
import React from 'react';

const ITEM_HEIGHT = 48;
const ITEM_PADDING_TOP = 8;
const MenuProps = {
  PaperProps: {
    style: {
      maxHeight: ITEM_HEIGHT * 7 + ITEM_PADDING_TOP,
      width: 250,
    },
  },
};

export interface CustomLookupFilterProps {
  lookup: Record<string, string>;
  selectedFilter: string[];
  onFilterChange: React.Dispatch<React.SetStateAction<string[]>>;
}

export const CustomLookupFilter = (props: CustomLookupFilterProps): React.ReactElement => {
  // FIXME - select closes every time a filter param is selected, which is bad for ux
  const { lookup, selectedFilter, onFilterChange } = props;
  return (
    <FormControl style={{ width: '100%', padding: '1rem' }}>
      <InputLabel htmlFor={'select-multiple-checkbox'}>Level Filter</InputLabel>
      <Select
        multiple
        value={selectedFilter}
        onChange={(event) => {
          if (typeof event.target.value === 'string') {
            onFilterChange([...selectedFilter, event.target.value]);
          } else {
            onFilterChange(event.target.value);
          }
        }}
        input={<Input id={'select-multiple-checkbox'} />}
        renderValue={(selecteds) =>
          (selecteds as string[]).map((selected: string) => lookup[selected]).join(', ')
        }
        MenuProps={MenuProps}
        style={{ marginTop: 0 }}
      >
        {Object.keys(lookup).map((key: string) => (
          <MenuItem key={key} value={key}>
            <Checkbox checked={selectedFilter.indexOf(key.toString()) > -1} />
            <ListItemText primary={lookup[key]} />
          </MenuItem>
        ))}
      </Select>
    </FormControl>
  );
};
