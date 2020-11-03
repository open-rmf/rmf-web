import { Divider, List, ListItem, Typography } from '@material-ui/core';
import React from 'react';
import Debug from 'debug';
import DashboardTooltip from './tooltip';
import { TooltipContext } from './app-contexts';

const debug = Debug('MainMenu');

export interface MainMenuProps {
  onDoorsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onLiftsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onRobotsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onDispensersClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onCommandsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
  onNegotiationsClick?(event: React.MouseEvent<HTMLDivElement, MouseEvent>): void;
}

export const MainMenu = React.memo((props: MainMenuProps) => {
  const { showTooltips } = React.useContext(TooltipContext);
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
        <DashboardTooltip
          title="This panel shows the commands that a user can request and RoMi will allocate the most suitable robot for the task"
          id="commands-tooltip"
          enabled={showTooltips}
        >
          <Typography variant="h5">Commands</Typography>
        </DashboardTooltip>
      </ListItem>
      <Divider />

      <ListItem data-item="Negotiations" button={true} onClick={props.onNegotiationsClick}>
        <DashboardTooltip
          title="This panel shows the negotiations between robots when there are conflicts in trajectories"
          id="negotiations-tooltip"
          enabled={showTooltips}
        >
          <Typography variant="h5">Negotiations</Typography>
        </DashboardTooltip>
      </ListItem>
    </List>
  );
});

export default MainMenu;
