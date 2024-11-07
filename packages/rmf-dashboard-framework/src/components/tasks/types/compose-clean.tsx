import { Autocomplete, TextField } from '@mui/material';
import React from 'react';

import { TaskBookingLabels } from '../booking-label';
import { TaskDefinition } from '../task-form';

export const ComposeCleanTaskDefinition: TaskDefinition = {
  taskDefinitionId: 'compose-clean',
  taskDisplayName: 'Clean',
  requestCategory: 'compose',
  scheduleEventColor: undefined,
};

interface GoToPlaceActivity {
  category: string;
  description: string;
}

interface CleanActivity {
  category: string;
  description: {
    unix_millis_action_duration_estimate: number;
    category: string;
    expected_finish_location: string;
    description: {
      zone: string;
    };
    use_tool_sink: boolean;
  };
}

export interface ComposeCleanTaskDescription {
  category: string;
  phases: [
    cleanPhase: {
      activity: {
        category: string;
        description: {
          activities: [goToPlaceActivity: GoToPlaceActivity, cleanActivity: CleanActivity];
        };
      };
    },
  ];
}

export function makeComposeCleanTaskBookingLabel(
  task_description: ComposeCleanTaskDescription,
): TaskBookingLabels {
  return {
    task_definition_id: ComposeCleanTaskDefinition.taskDefinitionId,
    destination:
      task_description.phases[0].activity.description.activities[1].description.description.zone,
  };
}

export function isComposeCleanTaskDescriptionValid(
  taskDescription: ComposeCleanTaskDescription,
): boolean {
  const goToPlaceActivity = taskDescription.phases[0].activity.description.activities[0];
  const cleanActivity = taskDescription.phases[0].activity.description.activities[1];
  return (
    goToPlaceActivity.description.length !== 0 &&
    cleanActivity.description.description.zone.length !== 0 &&
    goToPlaceActivity.description === cleanActivity.description.description.zone
  );
}

export function makeDefaultComposeCleanTaskDescription(): ComposeCleanTaskDescription {
  return {
    category: 'clean',
    phases: [
      {
        activity: {
          category: 'sequence',
          description: {
            activities: [
              {
                category: 'go_to_place',
                description: '',
              },
              {
                category: 'perform_action',
                description: {
                  unix_millis_action_duration_estimate: 60000,
                  category: 'clean',
                  expected_finish_location: '',
                  description: {
                    zone: '',
                  },
                  use_tool_sink: true,
                },
              },
            ],
          },
        },
      },
    ],
  };
}

export function makeComposeCleanTaskShortDescription(
  desc: ComposeCleanTaskDescription,
  displayName: string | undefined,
): string {
  const cleanActivity = desc.phases[0].activity.description.activities[1];
  return `[${displayName ?? ComposeCleanTaskDefinition.taskDisplayName}] zone [${
    cleanActivity.description.description.zone
  }]`;
}

interface ComposeCleanTaskFormProps {
  taskDesc: ComposeCleanTaskDescription;
  cleaningZones: string[];
  onChange(cleanTaskDescription: ComposeCleanTaskDescription): void;
  onValidate(valid: boolean): void;
}

export function ComposeCleanTaskForm({
  taskDesc,
  cleaningZones,
  onChange,
  onValidate,
}: ComposeCleanTaskFormProps): React.JSX.Element {
  const onInputChange = (desc: ComposeCleanTaskDescription) => {
    onValidate(isComposeCleanTaskDescriptionValid(desc));
    onChange(desc);
  };

  return (
    <Autocomplete
      id="cleaning-zone"
      freeSolo
      fullWidth
      options={cleaningZones}
      value={taskDesc.phases[0].activity.description.activities[0].description}
      onChange={(_ev, newValue) => {
        const zone = newValue ?? '';
        taskDesc.phases[0].activity.description.activities[0].description = zone;
        taskDesc.phases[0].activity.description.activities[1].description.expected_finish_location =
          zone;
        taskDesc.phases[0].activity.description.activities[1].description.description.zone = zone;
        onInputChange(taskDesc);
      }}
      onBlur={(ev) => {
        const zone = (ev.target as HTMLInputElement).value;
        taskDesc.phases[0].activity.description.activities[0].description = zone;
        taskDesc.phases[0].activity.description.activities[1].description.expected_finish_location =
          zone;
        taskDesc.phases[0].activity.description.activities[1].description.description.zone = zone;
        onInputChange(taskDesc);
      }}
      renderInput={(params) => <TextField {...params} label="Cleaning Zone" required={true} />}
    />
  );
}
