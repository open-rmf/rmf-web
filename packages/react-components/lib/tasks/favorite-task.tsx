import React from 'react';
import { TaskFavorite } from 'api-client';
import {
  Divider,
  Grid,
  IconButton,
  List,
  ListItem,
  ListItemSecondaryAction,
  ListItemText,
  Typography,
  useTheme,
} from '@mui/material';
import DeleteIcon from '@mui/icons-material/Delete';
import EditIcon from '@mui/icons-material/Edit';

export interface FavoriteTaskProps {
  listItemText: string;
  listItemClick: () => void;
}
export function FavoriteTask({ listItemText, listItemClick }: FavoriteTaskProps): JSX.Element {
  const theme = useTheme();
  return (
    <>
      <ListItem
        sx={{ width: theme.spacing(30) }}
        button
        onClick={listItemClick}
        role="listitem button"
        divider={true}
      >
        <ListItemText primary={listItemText} />
        <ListItemSecondaryAction>
          <IconButton aria-label="edit">
            <EditIcon />
          </IconButton>
          <IconButton edge="end" aria-label="delete">
            <DeleteIcon />
          </IconButton>
        </ListItemSecondaryAction>
      </ListItem>
    </>
  );
}
