import { Grid, TextField, useTheme } from '@mui/material';
import React from 'react';

import { TaskBookingLabels } from '../booking-label';
import { TaskDefinition } from '../task-form';

export const CustomComposeTaskDefinition: TaskDefinition = {
  taskDefinitionId: 'custom_compose',
  taskDisplayName: 'Custom Compose Task',
  requestCategory: 'compose',
  scheduleEventColor: undefined,
};

export type CustomComposeTaskDescription = string;

export function makeCustomComposeTaskBookingLabel(): TaskBookingLabels {
  return {
    task_definition_id: CustomComposeTaskDefinition.taskDefinitionId,
  };
}

export function makeCustomComposeTaskShortDescription(desc: CustomComposeTaskDescription): string {
  return desc;
}

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
  onValidate(valid: boolean): void;
}

export function CustomComposeTaskForm({
  taskDesc,
  onChange,
  onValidate,
}: CustomComposeTaskFormProps): React.JSX.Element {
  const theme = useTheme();
  const onInputChange = (desc: CustomComposeTaskDescription) => {
    onValidate(isCustomTaskDescriptionValid(desc));
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
