import { Grid, TextField, useTheme } from '@mui/material';
import React from 'react';

export type CustomComposeTaskDescription = string;

const isCustomTaskDescriptionValid = (taskDescription: string): boolean => {
  if (taskDescription.length === 0) {
    return false;
  }

  try {
    JSON.parse(taskDescription);
  } catch (e) {
    return false;
  }

  return true;
};

interface CustomComposeTaskFormProps {
  taskDesc: CustomComposeTaskDescription;
  onChange(customComposeTaskDescription: CustomComposeTaskDescription): void;
  allowSubmit(allow: boolean): void;
}

export function CustomComposeTaskForm({
  taskDesc,
  onChange,
  allowSubmit,
}: CustomComposeTaskFormProps) {
  const theme = useTheme();
  const onInputChange = (desc: CustomComposeTaskDescription) => {
    allowSubmit(isCustomTaskDescriptionValid(desc));
    onChange(desc);
  };

  return (
    <Grid container spacing={theme.spacing(2)}>
      <Grid item xs={12}>
        <TextField
          label="Multiline"
          multiline
          rows={8}
          value={taskDesc}
          fullWidth
          onChange={(ev) => {
            onInputChange(ev.target.value);
          }}
        />
      </Grid>
    </Grid>
  );
}
