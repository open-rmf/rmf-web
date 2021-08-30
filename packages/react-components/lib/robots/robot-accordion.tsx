import { Accordion, AccordionProps, makeStyles } from '@material-ui/core';
import * as RmfModels from 'rmf-models';
import Debug from 'debug';
import React from 'react';
import ItemAccordionDetails from '../item-accordion-details';
import ItemAccordionSummary from '../item-accordion-summary';
import { SimpleInfo } from '../simple-info';
import { robotModeToString, VerboseRobot } from './utils';
import { Map as LMap } from 'react-leaflet';
import { RobotInfo as RobotGraphics } from './robot-info';

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
    { name: 'Model', value: robot.model },
    { name: 'Fleet', value: fleetName },
    {
      name: 'Current Map',
      value: `${robot.location.level_name}`,
    },
    { name: 'Task Id', value: robot.task_id },
  ];

  return <SimpleInfo infoData={data} />;
};

export interface RobotAccordionProps extends Omit<AccordionProps, 'children'> {
  fleetName: string;
  robot: VerboseRobot;
  mapRef?: React.RefObject<LMap>;
}

export const RobotAccordion = React.forwardRef(
  (props: RobotAccordionProps, ref: React.Ref<HTMLElement>) => {
    const { fleetName, robot, mapRef, ...otherProps } = props;
    debug(`render ${robot.name}`);
    const classes = useStyles();

    function onAccordianClick(robot: VerboseRobot, mapRef?: React.RefObject<LMap>) {
      mapRef?.current?.leafletElement.setView([robot.state.location.y, robot.state.location.x], 5);
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
          <div style={{ padding: '1rem' }}>
            <RobotGraphics robot={robot} />
          </div>
        </ItemAccordionDetails>
      </Accordion>
    );
  },
);

export default RobotAccordion;
