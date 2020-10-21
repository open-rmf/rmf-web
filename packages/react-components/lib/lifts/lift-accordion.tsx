import { makeStyles } from '@material-ui/core';
import Accordion, { AccordionProps } from '@material-ui/core/Accordion';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import { AntTab, AntTabs, TabPanel } from '../ant-tab';
import ItemAccordionDetails from '../item-accordion-details';
import ItemAccordionSummary from '../item-accordion-summary';
import SimpleInfo, { SimpleInfoData } from '../simple-info';
import LiftRequestForm from './lift-request-form';
import {
  doorStateToString,
  liftModeToString,
  motionStateToString,
  requestDoorModes,
  requestModes,
} from './lift-utils';

const debug = Debug('Lifts:Accordion');

interface LiftInfoProps {
  lift: RomiCore.Lift;
  liftState?: RomiCore.LiftState;
}

const useStyles = makeStyles((theme) => ({
  disabledText: {
    color: theme.palette.action.disabled,
  },
  liftFloorLabelStopped: {
    borderColor: theme.palette.info.main,
  },
  liftFloorLabelMoving: {
    borderColor: theme.palette.warning.main,
  },
  liftFloorLabelUnknown: {
    borderColor: '#cccccc',
  },
}));

const LiftInfo = (props: LiftInfoProps) => {
  const { lift, liftState } = props;
  const classes = useStyles();

  const data = [
    { name: 'Name', value: lift.name },
    { name: 'Location', value: `(${lift.ref_x.toFixed(3)}, ${lift.ref_y.toFixed(3)})` },
    { name: 'Destination Floor', value: liftState ? liftState.destination_floor : 'Unknown' },
    { name: 'Available Floors', value: lift.levels },
    {
      name: 'Current Mode',
      value: liftState ? liftModeToString(liftState.current_mode) : 'Unknown',
      className: !liftState ? classes.disabledText : undefined,
    },
    {
      name: 'Available Modes',
      value: liftState
        ? Array.from(liftState.available_modes).map((mode) => liftModeToString(mode))
        : 'Unknown',
      className: !liftState ? classes.disabledText : undefined,
    },
    {
      name: 'Door State',
      value: liftState ? doorStateToString(liftState.door_state) : 'Unknown',
      className: !liftState ? classes.disabledText : undefined,
    },
    {
      name: 'Motion State',
      value: liftState ? motionStateToString(liftState.motion_state) : 'Unknown',
      className: !liftState ? classes.disabledText : undefined,
    },
  ] as SimpleInfoData[];

  return <SimpleInfo data={data} />;
};

export interface LiftAccordionProps extends Omit<AccordionProps, 'children'> {
  lift: RomiCore.Lift;
  liftState?: RomiCore.LiftState;
  onRequest?(
    lift: RomiCore.Lift,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
}

export const LiftAccordion = React.memo((props: LiftAccordionProps) => {
  const { lift, liftState, onRequest, ...otherProps } = props;
  debug(`render ${lift.name}`);
  const [tabValue, setTabValue] = React.useState(0);
  const classes = useStyles();

  const liftFloorLabelClass = React.useCallback(
    (liftState?: RomiCore.LiftState): string => {
      if (!liftState) {
        return classes.liftFloorLabelUnknown;
      }
      switch (liftState.motion_state) {
        case RomiCore.LiftState.MOTION_UP:
        case RomiCore.LiftState.MOTION_DOWN:
          return classes.liftFloorLabelMoving;
        case RomiCore.LiftState.MOTION_STOPPED:
          return classes.liftFloorLabelStopped;
        default:
          return classes.liftFloorLabelUnknown;
      }
    },
    [classes],
  );

  const handleTabChange = React.useCallback(
    (_event: React.ChangeEvent<unknown>, newValue: number) => {
      setTabValue(newValue);
    },
    [],
  );

  return (
    <Accordion {...otherProps}>
      <ItemAccordionSummary
        title={lift.name}
        status={liftState ? liftState.current_floor : 'N/A'}
        classes={{ status: liftFloorLabelClass(liftState) }}
      />
      <ItemAccordionDetails>
        <AntTabs variant="fullWidth" value={tabValue} onChange={handleTabChange}>
          <AntTab label="Info" />
          <AntTab label="Request" />
        </AntTabs>
        <TabPanel value={tabValue} index={0}>
          <LiftInfo lift={lift} liftState={liftState} />
        </TabPanel>
        <TabPanel value={tabValue} index={1}>
          {lift.levels && (
            <LiftRequestForm
              lift={lift}
              availableDoorModes={requestDoorModes}
              availableRequestTypes={requestModes}
              onRequest={onRequest}
            />
          )}
        </TabPanel>
      </ItemAccordionDetails>
    </Accordion>
  );
});

export default LiftAccordion;
