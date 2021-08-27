import { Accordion, AccordionProps, makeStyles } from '@material-ui/core';
import * as RmfModels from 'rmf-models';
import Debug from 'debug';
import React from 'react';
import ItemAccordionDetails from '../item-accordion-details';
import ItemAccordionSummary from '../item-accordion-summary';
import { SimpleInfo } from '../simple-info';
import { robotModeToString, VerboseRobot } from './utils';
import { Map as LMap } from 'react-leaflet';

const debug = Debug('Robots:RobotAccordion');

const useStyles = makeStyles((theme) => ({
  robotStatusLabel: {
    borderColor: theme.palette.info.main,
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
      value: `${robot.location.level_name}`,
    },
    { name: 'Task Id', value: robot.task_id },
    { name: 'Battery', value: robot.battery_percent.toFixed(0) },
  ];

  return <SimpleInfo infoData={data} />;
};

export interface RobotAccordionProps extends Omit<AccordionProps, 'children'> {
  fleetName: string;
  robot: VerboseRobot;
  mapRef?: React.RefObject<LMap>;
  onRobotSelect?: React.Dispatch<React.SetStateAction<VerboseRobot | undefined>>;
}

export const RobotAccordion = React.forwardRef(
  (props: RobotAccordionProps, ref: React.Ref<HTMLElement>) => {
    const { fleetName, robot, mapRef, onRobotSelect, ...otherProps } = props;
    debug(`render ${robot.name}`);
    const classes = useStyles();

    function onAccordianClick(robot: VerboseRobot, mapRef?: React.RefObject<LMap>) {
      mapRef?.current?.leafletElement.setView([robot.state.location.y, robot.state.location.x], 5);
      onRobotSelect && onRobotSelect(robot);
    }

    return (
      <Accordion ref={ref} {...otherProps}>
        <ItemAccordionSummary
          title={robot.name}
          statusProps={{
            className: classes.robotStatusLabel,
            text: robotModeToString(robot.state.mode),
          }}
          onAccordianClick={() => onAccordianClick(robot, mapRef)}
        />
        <ItemAccordionDetails>
          <RobotInfo fleetName={fleetName} robot={robot.state} />
        </ItemAccordionDetails>
      </Accordion>
    );
  },
);

export default RobotAccordion;
