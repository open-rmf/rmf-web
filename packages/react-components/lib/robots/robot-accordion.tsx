import { Accordion, AccordionProps, makeStyles } from '@material-ui/core';
import * as RmfModels from 'rmf-models';
import Debug from 'debug';
import React from 'react';
import ItemAccordionDetails from '../item-accordion-details';
import ItemAccordionSummary from '../item-accordion-summary';
import { SimpleInfo } from '../simple-info';
import { robotModeToString } from './utils';

const debug = Debug('Robots:RobotAccordion');

const useStyles = makeStyles((theme) => ({
  robotStatusLabel: {
    borderColor: theme.palette.info.main,
  },
  root: {
    backgroundColor: theme.secondaryBackground,
    color: theme.fontColors,
  },
}));

interface RobotInfoProps {
  fleetName: string;
  robot: RmfModels.RobotState;
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
  robot: RmfModels.RobotState;
}

export const RobotAccordion = React.forwardRef(
  (props: RobotAccordionProps, ref: React.Ref<HTMLElement>) => {
    const { fleetName, robot, ...otherProps } = props;
    debug(`render ${robot.name}`);
    const classes = useStyles();

    return (
      <Accordion ref={ref} {...otherProps} className={classes.root}>
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
