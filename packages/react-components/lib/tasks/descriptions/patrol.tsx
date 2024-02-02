import DeleteIcon from '@mui/icons-material/Delete';
import PlaceOutlined from '@mui/icons-material/PlaceOutlined';
import { PositiveIntField } from '../../form-inputs';
import React from 'react';
import {
  Autocomplete,
  Grid,
  IconButton,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  TextField,
  useTheme,
} from '@mui/material';

export interface PatrolTaskDescription {
  category: string;
  phases: [
    patrolPhase: {
      activity: {
        category: string;
        description: {
          activities: [
            patrolActivity: {
              places: string[];
              rounds: number;
            },
          ];
        };
      };
    },
  ];
}

export function isPatrolTaskDescriptionValid(taskDescription: PatrolTaskDescription): boolean {
  const patrolActivity = taskDescription.phases[0].activity.description.activities[0];
  if (patrolActivity.places.length === 0) {
    return false;
  }
  for (const place of patrolActivity.places) {
    if (place.length === 0) {
      return false;
    }
  }
  return patrolActivity.rounds > 0;
}

export function makeDefaultPatrolTaskDescription(): PatrolTaskDescription {
  return {
    category: 'patrol',
    phases: [
      {
        activity: {
          category: 'sequence',
          description: {
            activities: [
              {
                places: [],
                rounds: 1,
              },
            ],
          },
        },
      },
    ],
  };
}

interface PlaceListProps {
  places: string[];
  onClick(places_index: number): void;
}

function PlaceList({ places, onClick }: PlaceListProps) {
  const theme = useTheme();
  return (
    <List
      dense
      sx={{
        bgcolor: 'background.paper',
        marginLeft: theme.spacing(3),
        marginRight: theme.spacing(3),
      }}
    >
      {places.map((value, index) => (
        <ListItem
          key={`${value}-${index}`}
          secondaryAction={
            <IconButton edge="end" aria-label="delete" onClick={() => onClick(index)}>
              <DeleteIcon />
            </IconButton>
          }
        >
          <ListItemIcon>
            <PlaceOutlined />
          </ListItemIcon>
          <ListItemText primary={`Place Name:   ${value}`} />
        </ListItem>
      ))}
    </List>
  );
}

interface PatrolTaskFormProps {
  taskDesc: PatrolTaskDescription;
  patrolWaypoints: string[];
  onChange(patrolTaskDescription: PatrolTaskDescription): void;
  allowSubmit(allow: boolean): void;
}

export function PatrolTaskForm({
  taskDesc,
  patrolWaypoints,
  onChange,
  allowSubmit,
}: PatrolTaskFormProps) {
  const theme = useTheme();
  const onInputChange = (desc: PatrolTaskDescription) => {
    allowSubmit(isPatrolTaskDescriptionValid(desc));
    onChange(desc);
  };

  return (
    <Grid container spacing={theme.spacing(2)} justifyContent="center" alignItems="center">
      <Grid item xs={10}>
        <Autocomplete
          id="place-input"
          freeSolo
          fullWidth
          options={patrolWaypoints}
          onChange={(_ev, newValue) => {
            if (!newValue) {
              return;
            }
            taskDesc.phases[0].activity.description.activities[0].places.push(newValue);
            onInputChange(taskDesc);
          }}
          renderInput={(params) => <TextField {...params} label="Place Name" required={true} />}
        />
      </Grid>
      <Grid item xs={2}>
        <PositiveIntField
          id="loops"
          label="Loops"
          value={taskDesc.phases[0].activity.description.activities[0].rounds}
          onChange={(_ev, val) => {
            taskDesc.phases[0].activity.description.activities[0].rounds = val;
            onInputChange(taskDesc);
          }}
        />
      </Grid>
      <Grid item xs={10}>
        <PlaceList
          places={taskDesc.phases[0].activity.description.activities[0].places}
          onClick={(places_index) =>
            taskDesc.phases[0].activity.description.activities[0].places.splice(places_index, 1) &&
            onInputChange({
              ...taskDesc,
            })
          }
        />
      </Grid>
    </Grid>
  );
}
