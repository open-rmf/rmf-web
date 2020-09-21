import {
  Accordion,
  AccordionDetails,
  AccordionProps,
  AccordionSummary,
  makeStyles,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import OmniPanelStatusLabels from './omni-panel-status-labels';
import { RobotInformation } from './robot-item-information';

const debug = Debug('OmniPanel:RobotItem');

export interface RobotItemProps extends Omit<AccordionProps, 'children'> {
  fleetName: string;
  robot: Readonly<RomiCore.RobotState>;
  onRobotClick?(robot: RomiCore.RobotState): void;
}

export const RobotItem = React.memo(
  React.forwardRef(function (
    props: RobotItemProps,
    ref: React.Ref<HTMLElement>,
  ): React.ReactElement {
    const { robot, onRobotClick, fleetName, ...otherProps } = props;
    const classes = useStyles();

    debug('render %s', robot.name);

    return (
      <Accordion ref={ref} data-component="RobotItem" data-name={robot.name} {...otherProps}>
        <AccordionSummary
          classes={{ content: classes.accordionSummaryContent }}
          expandIcon={<ExpandMoreIcon />}
        >
          <OmniPanelStatusLabels
            modalLabelClass={classes.robotStatusLabel}
            name={robot.name}
            modeText={robotModeToString(robot.mode)}
          />
        </AccordionSummary>
        <AccordionDetails data-role="details" className={classes.accordionDetail}>
          <RobotInformation robot={robot} />
        </AccordionDetails>
      </Accordion>
    );
  }),
);

export default RobotItem;

const useStyles = makeStyles((theme) => ({
  accordionSummaryContent: {
    alignItems: 'center',
    justifyContent: 'space-between',
  },

  accordionDetail: {
    flexFlow: 'column',
    overflowX: 'auto',
  },

  accordionDetailLine: {
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
