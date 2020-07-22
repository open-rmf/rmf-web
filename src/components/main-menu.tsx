import { Divider, List, ListItem, Typography } from '@material-ui/core';
import React from 'react';

export interface MainMenuProps {
  onDoorsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onLiftsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onRobotsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onDispensersClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onCommandsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
}

export default function MainMenu(props: MainMenuProps): React.ReactElement {
  return (
    <List data-component="MainMenu">
      <ListItem data-item="Doors" button={true} onClick={props.onDoorsClick}>
        <Typography variant="h5">Doors</Typography>
      </ListItem>

      <Divider />

      <ListItem data-item="Lifts" button={true} onClick={props.onLiftsClick}>
        <Typography variant="h5">Lifts</Typography>
      </ListItem>

      <Divider />

      <ListItem data-item="Robots" button={true} onClick={props.onRobotsClick}>
        <Typography variant="h5">Robots</Typography>
      </ListItem>

      <Divider />

      <ListItem data-item="Dispensers" button={true} onClick={props.onDispensersClick}>
        <Typography variant="h5">Dispensers</Typography>
      </ListItem>

      <Divider />

      <ListItem data-item="Commands" button={true} onClick={props.onCommandsClick}>
        <Typography variant="h5">Commands</Typography>
      </ListItem>
    </List>
  );
}