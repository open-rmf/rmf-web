import { Accordion, AccordionProps, makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import ItemAccordionDetails from '../item-accordion-details';
import ItemAccordionSummary from '../item-accordion-summary';
import { SimpleInfo } from '../simple-info';

const debug = Debug('Robots:RobotAccordion');

const useStyles = makeStyles((theme) => ({
  robotStatusLabel: {
    borderColor: theme.palette.info.main,
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

interface RobotInfoProps {
  fleetName: string;
  robot: RomiCore.RobotState;
}

const RobotInfo = (props: RobotInfoProps) => {
  const { fleetName, robot } = props;

  const data = [
    { name: 'Name', value: robot.name },
    { name: 'Model', value: robot.model },
    { name: 'Fleet', value: fleetName },
    {
      name: 'Location',
      value: `${robot.location.level_name} (${robot.location.x.toFixed(
        3,
      )}, ${robot.location.y.toFixed(3)})`,
    },
    { name: 'Yaw', value: robot.location.yaw.toFixed(3) },
    { name: 'Task Id', value: robot.task_id },
    { name: 'Battery', value: robot.battery_percent.toFixed(0) },
  ];

  return <SimpleInfo infoData={data} />;
};

export interface RobotAccordionProps extends Omit<AccordionProps, 'children'> {
  fleetName: string;
  robot: RomiCore.RobotState;
}

export const RobotAccordion = React.forwardRef(
  (props: RobotAccordionProps, ref: React.Ref<HTMLElement>) => {
    const { fleetName, robot, ...otherProps } = props;
    debug(`render ${robot.name}`);
    const classes = useStyles();

    return (
      <Accordion ref={ref} {...otherProps}>
        <ItemAccordionSummary
          title={robot.name}
          statusProps={{
            className: classes.robotStatusLabel,
            text: robotModeToString(robot.mode),
          }}
        />
        <ItemAccordionDetails>
          <RobotInfo fleetName={fleetName} robot={robot} />
        </ItemAccordionDetails>
      </Accordion>
    );
  },
);

export default RobotAccordion;
