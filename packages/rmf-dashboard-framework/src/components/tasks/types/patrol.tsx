import DeleteIcon from '@mui/icons-material/Delete';
import PlaceOutlined from '@mui/icons-material/PlaceOutlined';
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
import React from 'react';

import { PositiveIntField } from '../../form-inputs';
import { TaskBookingLabels } from '../booking-label';
import { TaskDefinition } from '../task-form';

export const PatrolTaskDefinition: TaskDefinition = {
  taskDefinitionId: 'patrol',
  taskDisplayName: 'Patrol',
  requestCategory: 'patrol',
  scheduleEventColor: undefined,
};

export interface PatrolTaskDescription {
  places: string[];
  rounds: number;
}

export function makePatrolTaskBookingLabel(
  task_description: PatrolTaskDescription,
): TaskBookingLabels {
  return {
    task_definition_id: PatrolTaskDefinition.taskDefinitionId,
    destination: task_description.places[task_description.places.length - 1],
  };
}

export const isPatrolTaskDescriptionValid = (taskDescription: PatrolTaskDescription): boolean => {
  if (taskDescription.places.length === 0) {
    return false;
  }
  for (const place of taskDescription.places) {
    if (place.length === 0) {
      return false;
    }
  }
  return taskDescription.rounds > 0;
};

export function makeDefaultPatrolTaskDescription(): PatrolTaskDescription {
  return {
    places: [],
    rounds: 1,
  };
}

export function makePatrolTaskShortDescription(
  desc: PatrolTaskDescription,
  displayName?: string,
): string {
  console.log(desc);

  const formattedPlaces = desc.places.map((place: string) => `[${place}]`);
  return `[${displayName ?? PatrolTaskDefinition.taskDisplayName}] [${
    desc.rounds
  }] round/s, along ${formattedPlaces.join(', ')}`;
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
          data-testid={`${value}-${index}`}
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

export function addPlaceToPatrolTaskDescription(
  taskDesc: PatrolTaskDescription,
  place: string,
): PatrolTaskDescription {
  const updatedTaskDesc = {
    ...taskDesc,
    places: taskDesc.places.concat(place).filter((el: string) => el),
  };
  return updatedTaskDesc;
}

interface PatrolTaskFormProps {
  taskDesc: PatrolTaskDescription;
  patrolWaypoints: string[];
  onChange(patrolTaskDescription: PatrolTaskDescription): void;
  onValidate(valid: boolean): void;
}

export function PatrolTaskForm({
  taskDesc,
  patrolWaypoints,
  onChange,
  onValidate,
}: PatrolTaskFormProps): React.JSX.Element {
  const theme = useTheme();
  const onInputChange = (desc: PatrolTaskDescription) => {
    onValidate(isPatrolTaskDescriptionValid(desc));
    onChange(desc);
  };

  React.useEffect(() => {
    onValidate(isPatrolTaskDescriptionValid(taskDesc));
  }, [onValidate, taskDesc]);

  return (
    <Grid container spacing={theme.spacing(2)} justifyContent="center" alignItems="center">
      <Grid item xs={10}>
        <Autocomplete
          id="place-input"
          data-testid="place-name"
          freeSolo
          fullWidth
          options={patrolWaypoints.sort()}
          onChange={(_ev, newValue) =>
            newValue !== null && onInputChange(addPlaceToPatrolTaskDescription(taskDesc, newValue))
          }
          sx={{
            '& .MuiOutlinedInput-root': {
              height: '3.5rem',
              fontSize: 20,
            },
          }}
          renderInput={(params) => (
            <TextField
              {...params}
              label="Place Name"
              required={true}
              InputLabelProps={{ style: { fontSize: 20 } }}
            />
          )}
        />
      </Grid>
      <Grid item xs={2}>
        <PositiveIntField
          id="loops"
          label="Loops"
          sx={{
            '& .MuiOutlinedInput-root': {
              height: '3.5rem',
              fontSize: 20,
            },
          }}
          value={taskDesc.rounds}
          onChange={(_ev, val) => {
            onInputChange({
              ...taskDesc,
              rounds: val,
            });
          }}
        />
      </Grid>
      <Grid item xs={10}>
        <PlaceList
          places={taskDesc && taskDesc.places ? taskDesc.places : []}
          onClick={(places_index) =>
            taskDesc.places.splice(places_index, 1) &&
            onInputChange({
              ...taskDesc,
            })
          }
        />
      </Grid>
    </Grid>
  );
}
