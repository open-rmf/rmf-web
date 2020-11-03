import { Divider, List, ListItem, Typography } from '@material-ui/core';
import React from 'react';
import Debug from 'debug';
import { MainMenuActionType } from './reducers/main-menu-reducer';
import { OmniPanelViewIndex } from './dashboard';

const debug = Debug('MainMenu');

export interface MainMenuProps {
  mainMenuStateHandler: any;
  onDoorsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onLiftsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onRobotsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onDispensersClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onCommandsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onNegotiationsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
}

export const MainMenu = React.memo((props: MainMenuProps) => {
  debug('render');
  const { mainMenuStateHandler } = props;

  const onCommandsClick = () => {
    mainMenuStateHandler({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Commands,
    });
  };

  const onDispensersClick = () => {
    mainMenuStateHandler({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Dispensers,
    });
  };

  const onDoorsClick = () => {
    mainMenuStateHandler({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Doors,
    });
  };

  const onLiftsClick = () => {
    mainMenuStateHandler({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Lifts,
    });
  };

  const onNegotiationsClick = () => {
    mainMenuStateHandler({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Negotiations,
    });
  };

  const onRobotsClick = () => {
    mainMenuStateHandler({
      type: MainMenuActionType.CURRENT_VIEW,
      payload: OmniPanelViewIndex.Robots,
    });
  };

  return (
    <List data-component="MainMenu">
      <ListItem data-item="Doors" button={true} onClick={onDoorsClick}>
        <Typography variant="h5">Doors</Typography>
      </ListItem>

      <Divider />

      <ListItem data-item="Lifts" button={true} onClick={onLiftsClick}>
        <Typography variant="h5">Lifts</Typography>
      </ListItem>

      <Divider />

      <ListItem data-item="Robots" button={true} onClick={onRobotsClick}>
        <Typography variant="h5">Robots</Typography>
      </ListItem>

      <Divider />

      <ListItem data-item="Dispensers" button={true} onClick={onDispensersClick}>
        <Typography variant="h5">Dispensers</Typography>
      </ListItem>

      <Divider />

      <ListItem data-item="Commands" button={true} onClick={onCommandsClick}>
        <Typography variant="h5">Commands</Typography>
      </ListItem>
      <Divider />

      <ListItem data-item="Negotiations" button={true} onClick={onNegotiationsClick}>
        <Typography variant="h5">Negotiations</Typography>
      </ListItem>
    </List>
  );
});

export default MainMenu;
