import {
  Divider,
  ExpansionPanelDetails,
  ExpansionPanelSummary,
  makeStyles,
  Typography,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import SpotlightExpansionPanel, { SpotlightValue } from './spotlight-expansion-panel';

export interface RobotsPanelProps {
  fleets: readonly RomiCore.FleetState[];
  spotlight?: SpotlightValue<string>;
}

const useStyles = makeStyles(theme => ({
  expansionSummaryContent: {
    alignItems: 'center',
    justifyContent: 'space-between',
  },

  expansionDetail: {
    flexFlow: 'column',
  },

  expansionDetailLine: {
    display: 'inline-flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },

  robotStatusLabel: {
    borderRadius: theme.shape.borderRadius,
    borderStyle: 'solid',
    borderColor: theme.palette.info.main,
    border: 2,
    padding: 5,
    minWidth: '4rem',
    textAlign: 'center',
  },
}));

function robotModeToString(robotMode: RomiCore.RobotMode): string {
  switch (robotMode.mode) {
    case RomiCore.RobotMode.MODE_CHARGING:
      return 'Charging';
    case RomiCore.RobotMode.MODE_DOCKING:
      return 'Docking';
    case RomiCore.RobotMode.MODE_EMERGENCY:
      return 'Emergency';
    case RomiCore.RobotMode.MODE_GOING_HOME:
      return 'Going Home';
    case RomiCore.RobotMode.MODE_IDLE:
      return 'Idle';
    case RomiCore.RobotMode.MODE_MOVING:
      return 'Moving';
    case RomiCore.RobotMode.MODE_PAUSED:
      return 'Paused';
    case RomiCore.RobotMode.MODE_WAITING:
      return 'Waiting';
    default:
      return `Unknown (${robotMode.mode})`;
  }
}

export default function RobotsPanel(props: RobotsPanelProps): React.ReactElement {
  console.log('a');
  const classes = useStyles();

  const robots = props.fleets
    .flatMap(fleet => fleet.robots)
    .map(robot => {
      return (
        <SpotlightExpansionPanel
          key={robot.name}
          index={robot.name}
          spotlight={props.spotlight}
          TransitionProps={{ unmountOnExit: true }}
        >
          <ExpansionPanelSummary
            classes={{ content: classes.expansionSummaryContent }}
            expandIcon={<ExpandMoreIcon />}
          >
            <Typography variant="h5">{robot.name}</Typography>
            <Typography className={classes.robotStatusLabel} variant="button">
              {robotModeToString(robot.mode)}
            </Typography>
          </ExpansionPanelSummary>
          <ExpansionPanelDetails className={classes.expansionDetail}>
            <div className={classes.expansionDetailLine}>
              <Typography variant="body1">Model:</Typography>
              <Typography variant="body1">{robot.model}</Typography>
            </div>
            <Divider />
            <div className={classes.expansionDetailLine}>
              <Typography variant="body1">Level:</Typography>
              <Typography variant="body1">{robot.location.level_name}</Typography>
            </div>
            <Divider />
            <div className={classes.expansionDetailLine}>
              <Typography variant="body1">Position:</Typography>
              <Typography variant="body1">
                {`(${robot.location.x.toFixed(3)}, ${robot.location.y.toFixed(3)})`}
              </Typography>
            </div>
            <Divider />
            <div className={classes.expansionDetailLine}>
              <Typography variant="body1">Yaw:</Typography>
              <Typography variant="body1">{robot.location.yaw.toFixed(3)}</Typography>
            </div>
            <Divider />
            <div className={classes.expansionDetailLine}>
              <Typography variant="body1">Task Id:</Typography>
              <Typography variant="body1" noWrap>
                {robot.task_id}
              </Typography>
            </div>
            <Divider />
            <div className={classes.expansionDetailLine}>
              <Typography variant="body1">Battery:</Typography>
              <Typography variant="body1">{robot.battery_percent}</Typography>
            </div>
          </ExpansionPanelDetails>
        </SpotlightExpansionPanel>
      );
    });

  console.log('b');
  return <React.Fragment>{robots}</React.Fragment>;
}
