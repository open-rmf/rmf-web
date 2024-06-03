import { Grid, TextField, useTheme } from '@mui/material';
import React from 'react';
import type { TaskBookingLabel } from 'api-client';
import { TaskDefinition } from '../create-task';

export const DefaultCustomComposeTaskDefinition: TaskDefinition = {
  taskDefinitionId: 'custom_compose',
  taskDisplayName: 'Custom Compose Task',
  requestCategory: 'compose',
};

export type CustomComposeTaskDescription = string;

export function makeCustomComposeTaskBookingLabel(): TaskBookingLabel {
  return {
    description: {
      task_definition_id: DefaultCustomComposeTaskDefinition.taskDefinitionId,
    },
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
  allowSubmit(allow: boolean): void;
}

export function CustomComposeTaskForm({
  taskDesc,
  onChange,
  allowSubmit,
}: CustomComposeTaskFormProps): React.JSX.Element {
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
