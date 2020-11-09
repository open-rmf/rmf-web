import { Divider, List, ListItem, Typography } from '@material-ui/core';
import React from 'react';
import Debug from 'debug';

const debug = Debug('MainMenu');

export interface MainMenuProps {
  onDoorsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onLiftsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onRobotsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onDispensersClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onCommandsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onNegotiationsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onTasksClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
}

export const MainMenu = React.memo((props: MainMenuProps) => {
  debug('render');

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
      <Divider />

      <ListItem data-item="Negotiations" button={true} onClick={props.onNegotiationsClick}>
        <Typography variant="h5">Negotiations</Typography>
      </ListItem>
      <Divider />

      <ListItem data-item="Plans" button={true} onClick={props.onTasksClick}>
        <Typography variant="h5">Plans</Typography>
      </ListItem>
      <Divider />
    </List>
  );
});

export default MainMenu;
