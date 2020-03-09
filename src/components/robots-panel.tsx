import {
  Divider,
  ExpansionPanel,
  ExpansionPanelDetails,
  ExpansionPanelSummary,
  makeStyles,
  Typography,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

interface RobotsPanelProps {
  fleets: readonly RomiCore.FleetState[];
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

  expansionDetailSubLine: {
    display: 'inline-flex',
    justifyContent: 'space-between',
    paddingLeft: theme.spacing(2),
    width: '100%',
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

export default function RobotsPanel(props: RobotsPanelProps): JSX.Element {
  const classes = useStyles();

  function renderLocation(location: RomiCore.Location): JSX.Element {
    const position = `(${location.x.toFixed(3)}, ${location.y.toFixed(3)})`;
    return (
      <React.Fragment>
        <Typography className={classes.expansionDetailSubLine} variant="body1">
          <span>Level:</span>
          <span>{location.level_name}</span>
        </Typography>
        <Typography className={classes.expansionDetailSubLine} variant="body1">
          <span>Pos:</span>
          <span>{position}</span>
        </Typography>
        <Typography className={classes.expansionDetailSubLine} variant="body1">
          <span>Yaw:</span>
          <span>{location.yaw.toFixed(3)}</span>
        </Typography>
      </React.Fragment>
    );
  }

  const robots = props.fleets
    .flatMap(fleet => fleet.robots)
    .map(robot => {
      return (
        <ExpansionPanel key={robot.name}>
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
            <div>
              <Typography variant="body1">Location:</Typography>
              {renderLocation(robot.location)}
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
        </ExpansionPanel>
      );
    });

  return <React.Fragment>{robots}</React.Fragment>;
}
