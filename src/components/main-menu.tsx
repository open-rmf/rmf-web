import { Divider, List, ListItem, Typography } from '@material-ui/core';
import React from 'react';

export interface MainMenuProps {
  onDoorsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onLiftsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onRobotsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onPlacesClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onDispensersClick?: (event: React.MouseEvent<HTMLButtonElement, MouseEvent>) => void;
}

export default function MainMenu(props: MainMenuProps): React.ReactElement {
  return (
    <List>
      <ListItem button={true} onClick={props.onDoorsClick}>
        <Typography variant="h5">Doors</Typography>
      </ListItem>

      <Divider />

      <ListItem button={true} onClick={props.onLiftsClick}>
        <Typography variant="h5">Lifts</Typography>
      </ListItem>

      <Divider />

      <ListItem button={true} onClick={props.onRobotsClick}>
        <Typography variant="h5">Robots</Typography>
      </ListItem>

      <Divider />

      <ListItem button={true} onClick={props.onPlacesClick}>
        <Typography variant="h5">Places</Typography>
      </ListItem>

      <ListItem button={true} onClick={props.onDispensersClick}>
        <Typography variant="h5">Dispensers</Typography>
      </ListItem>
    </List>
  );
}
