import { Autocomplete, TextField } from '@mui/material';
import React from 'react';

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

export interface CleanTaskDescription {
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

export function isCleanTaskDescriptionValid(taskDescription: CleanTaskDescription): boolean {
  const goToPlaceActivity = taskDescription.phases[0].activity.description.activities[0];
  const cleanActivity = taskDescription.phases[0].activity.description.activities[1];
  return (
    goToPlaceActivity.description.length !== 0 &&
    cleanActivity.description.description.zone.length !== 0 &&
    goToPlaceActivity.description === cleanActivity.description.description.zone
  );
}

export function makeDefaultCleanTaskDescription(): CleanTaskDescription {
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

interface CleanTaskFormProps {
  taskDesc: CleanTaskDescription;
  cleaningZones: string[];
  onChange(cleanTaskDescription: CleanTaskDescription): void;
  allowSubmit(allow: boolean): void;
}

export function CleanTaskForm({
  taskDesc,
  cleaningZones,
  onChange,
  allowSubmit,
}: CleanTaskFormProps): React.JSX.Element {
  const onInputChange = (desc: CleanTaskDescription) => {
    allowSubmit(isCleanTaskDescriptionValid(desc));
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
