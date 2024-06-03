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
  useMediaQuery,
  useTheme,
} from '@mui/material';
import React from 'react';
import { PositiveIntField } from '../../form-inputs';
import { TaskDefinition } from '../create-task';
import type { TaskBookingLabel } from 'api-client';

export const DefaultPatrolTaskDefinition: TaskDefinition = {
  task_definition_id: 'patrol',
  task_display_name: 'Patrol',
};

export interface PatrolTaskDescription {
  places: string[];
  rounds: number;
}

export function makePatrolTaskBookingLabel(
  task_description: PatrolTaskDescription,
): TaskBookingLabel {
  return {
    description: {
      task_definition_id: 'patrol',
      destination: task_description.places[task_description.places.length - 1],
    },
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

export function makeDefaultPatrolTask(): PatrolTaskDescription {
  return {
    places: [],
    rounds: 1,
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
}: PatrolTaskFormProps): React.JSX.Element {
  const theme = useTheme();
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  const onInputChange = (desc: PatrolTaskDescription) => {
    allowSubmit(isPatrolTaskDescriptionValid(desc));
    onChange(desc);
  };
  allowSubmit(isPatrolTaskDescriptionValid(taskDesc));

  return (
    <Grid container spacing={theme.spacing(2)} justifyContent="center" alignItems="center">
      <Grid item xs={isScreenHeightLessThan800 ? 8 : 10}>
        <Autocomplete
          id="place-input"
          freeSolo
          fullWidth
          options={patrolWaypoints.sort()}
          onChange={(_ev, newValue) =>
            newValue !== null &&
            onInputChange({
              ...taskDesc,
              places: taskDesc.places.concat(newValue).filter((el: string) => el),
            })
          }
          sx={{
            '& .MuiOutlinedInput-root': {
              height: isScreenHeightLessThan800 ? '3rem' : '3.5rem',
              fontSize: isScreenHeightLessThan800 ? 14 : 20,
            },
          }}
          renderInput={(params) => (
            <TextField
              {...params}
              label="Place Name"
              required={true}
              InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 14 : 20 } }}
            />
          )}
        />
      </Grid>
      <Grid item xs={isScreenHeightLessThan800 ? 4 : 2}>
        <PositiveIntField
          id="loops"
          label="Loops"
          sx={{
            '& .MuiOutlinedInput-root': {
              height: isScreenHeightLessThan800 ? '3rem' : '3.5rem',
              fontSize: isScreenHeightLessThan800 ? 14 : 20,
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
