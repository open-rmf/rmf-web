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
  places: string[];
  rounds: number;
}

function isPatrolTaskDescriptionValid(taskDescription: PatrolTaskDescription): boolean {
  if (taskDescription.places.length === 0) {
    return false;
  }
  for (const place of taskDescription.places) {
    if (place.length === 0) {
      return false;
    }
  }
  return taskDescription.rounds > 0;
}

export function makeDefaultPatrolTaskDescription(): PatrolTaskDescription {
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
          onChange={(_ev, newValue) =>
            newValue !== null &&
            onInputChange({
              ...taskDesc,
              places: taskDesc.places.concat(newValue).filter((el: string) => el),
            })
          }
          renderInput={(params) => <TextField {...params} label="Place Name" required={true} />}
        />
      </Grid>
      <Grid item xs={2}>
        <PositiveIntField
          id="loops"
          label="Loops"
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
