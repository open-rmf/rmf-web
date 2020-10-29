import { Divider, List, ListItem, makeStyles, Tooltip, Typography } from '@material-ui/core';
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
  tooltips: boolean;
}

export const MainMenu = React.memo((props: MainMenuProps) => {
  const { tooltips } = props;
  debug('render');
  const classes = useStyles();

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

      {!tooltips && (
        <>
          <ListItem data-item="Commands" button={true} onClick={props.onCommandsClick}>
            <Typography variant="h5">Commands</Typography>
          </ListItem>
          <Divider />

          <ListItem data-item="Negotiations" button={true} onClick={props.onNegotiationsClick}>
            <Typography variant="h5">Negotiations</Typography>
          </ListItem>
        </>
      )}

      {tooltips && (
        <>
          <ListItem data-item="Commands" button={true} onClick={props.onCommandsClick}>
            <Tooltip
              title="this panel shows the commands that a user can request and RoMi will allocate the most suitable robot for the task"
              arrow
              id="commands-tooltip"
              className={classes.tooltipWidth}
            >
              <Typography variant="h5">Commands</Typography>
            </Tooltip>
          </ListItem>
          <Divider />

          <ListItem data-item="Negotiations" button={true} onClick={props.onNegotiationsClick}>
            <Tooltip
              title="this panel shows the negotiations between robots when there are conflicts in trajectories"
              arrow
              id="negotiations-tooltip"
              className={classes.tooltipWidth}
            >
              <Typography variant="h5">Negotiations</Typography>
            </Tooltip>
          </ListItem>
        </>
      )}
    </List>
  );
});

const useStyles = makeStyles((theme) => ({
  tooltipWidth: {
    maxWidth: 200,
  },
}));

export default MainMenu;
