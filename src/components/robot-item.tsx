import {
  ExpansionPanel,
  ExpansionPanelDetails,
  ExpansionPanelProps,
  ExpansionPanelSummary,
  makeStyles,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import { RobotInformation } from './robot-item-information';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

import OmniPanelStatusLabels from './omni-panel-status-labels';

export interface RobotItemProps extends Omit<ExpansionPanelProps, 'children'> {
  fleetName: string;
  robot: Readonly<RomiCore.RobotState>;
  onRobotClick?(robot: RomiCore.RobotState): void;
}

export const RobotItem = React.forwardRef(function(
  props: RobotItemProps,
  ref: React.Ref<HTMLElement>,
): React.ReactElement {
  const { robot, onRobotClick, fleetName, ...otherProps } = props;
  const classes = useStyles();
  return (
    <ExpansionPanel ref={ref} data-component="RobotItem" data-name={robot.name} {...otherProps}>
      <ExpansionPanelSummary
        classes={{ content: classes.expansionSummaryContent }}
        expandIcon={<ExpandMoreIcon />}
      >
        <OmniPanelStatusLabels
          modalLabelClass={classes.robotStatusLabel}
          name={robot.name}
          modeText={robotModeToString(robot.mode)}
        />
      </ExpansionPanelSummary>
      <ExpansionPanelDetails data-role="details" className={classes.expansionDetail}>
        <RobotInformation robot={robot} />
      </ExpansionPanelDetails>
    </ExpansionPanel>
  );
});

export default RobotItem;

const useStyles = makeStyles(theme => ({
  expansionSummaryContent: {
    alignItems: 'center',
    justifyContent: 'space-between',
  },

  expansionDetail: {
    flexFlow: 'column',
    overflowX: 'auto',
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
