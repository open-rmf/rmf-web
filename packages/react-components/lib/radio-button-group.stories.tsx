import React from 'react';
import { Story, Meta } from '@storybook/react';
import { RadioButtonGroup } from './radio-button-group';

export default {
  title: 'Radio Group',
  component: RadioButtonGroup,
} as Meta;

export const RowRadioGroup: Story = ({ ...args }) => {
  return (
    <RadioButtonGroup
      row={args.row}
      options={args.options}
      formLabel={args.formLabel}
      radioGroupName={args.radioGroupName}
      onHandleChange={() => alert('clicked')}
      {...args}
    />
  );
};
RowRadioGroup.args = {
  formLabel: 'Row Group',
  radioGroupName: 'value',
  row: true,
  options: ['option 1', 'option 2', 'option 3'],
};

export const ColumnRadioGroup: Story = ({ ...args }) => {
  return (
    <RadioButtonGroup
      row={args.row}
      options={args.options}
      formLabel={args.formLabel}
      radioGroupName={args.radioGroupName}
      onHandleChange={() => alert('clicked')}
      {...args}
    />
  );
};
ColumnRadioGroup.args = {
  formLabel: 'Column Group',
  radioGroupName: 'value',
  options: ['option 1', 'option 2', 'option 3'],
};
