import {
  ExpansionPanel,
  ExpansionPanelDetails,
  ExpansionPanelProps,
  ExpansionPanelSummary,
  makeStyles,
  Typography,
  Tab,
  Tabs,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React, { CSSProperties } from 'react';
import { AntTab, AntTabs, TabPanel } from '../tab';
import { LiftInformation } from './lift-item-information';
import LiftRequestForm from './lift-item-form';
import { LiftRequestManager } from '../../lift-state-manager';

export interface LiftItemProps extends Omit<ExpansionPanelProps, 'children'>{
  id?: string;
  lift: Readonly<RomiCore.Lift>;
  liftState?: Readonly<RomiCore.LiftState>;
  enableRequest?: boolean;
  onRequest?(
    lift: RomiCore.Lift,
    door_state: number,
    request_type: number,
    destination: string,
  ): void;
}

export const LiftItem = React.forwardRef(function(
  props: LiftItemProps,
  ref: React.Ref<HTMLElement>,
): React.ReactElement {
  const { id, lift, liftState, enableRequest, onRequest, ...otherProps } = props;
  const [tabValue, setTabValue] = React.useState(0);
  const classes = useStyles();

  function liftFloorLabel(liftState?: RomiCore.LiftState): string {
    if (!liftState) {
      return classes.liftFloorLabelUnknown;
    }
    switch (liftState.motion_state) {
      case RomiCore.LiftState.MOTION_UP:
      case RomiCore.LiftState.MOTION_DOWN:
        return classes.liftFloorLabelMoving;
      default:
        return classes.liftFloorLabelStopped;
    }
  }

  function handleRequest(doorState: number, requestType: number, destination: string): void {
    !!onRequest && onRequest(lift, doorState, requestType, destination);
  }

  const handleChange = (event: React.ChangeEvent<{}>, newValue: number) => {
    setTabValue(newValue);
  };

  const doorStates = React.useMemo(() => LiftRequestManager.getDoorModes(), []);
  const requestTypes = React.useMemo(() => LiftRequestManager.getLiftRequestModes(), []);

  return (
    <ExpansionPanel ref={ref} id={id} {...otherProps}>
      <ExpansionPanelSummary
        classes={{ content: classes.expansionSummaryContent }}
        expandIcon={<ExpandMoreIcon />}
      >
        <Typography variant="h5">{lift.name}</Typography>
        <Typography className={liftFloorLabel(liftState)} variant="button">
          {liftState ? liftState.current_floor : 'Unknown'}
        </Typography>
      </ExpansionPanelSummary>
      <ExpansionPanelDetails className={classes.expansionDetail}>
        <AntTabs value={tabValue} onChange={handleChange} aria-label="scrollable auto tabs example">
          <AntTab label="Info" />
          <AntTab label="Request" />
        </AntTabs>
        <TabPanel value={tabValue} index={0}>
          <LiftInformation lift={lift} liftState={liftState} />
        </TabPanel>
        <TabPanel value={tabValue} index={1}>
          {lift.levels && (
            <LiftRequestForm
              liftRequest={handleRequest}
              doorStates={doorStates}
              requestTypes={requestTypes}
              destinationList={lift.levels}
            />
          )}
        </TabPanel>
      </ExpansionPanelDetails>
    </ExpansionPanel>
  );
});

const useStyles = makeStyles(theme => {
  const liftFloorLabelBase: CSSProperties = {
    borderRadius: theme.shape.borderRadius,
    borderStyle: 'solid',

    border: 2,
    padding: 5,
    width: '4rem',
    textAlign: 'center',
  };

  return {
    expansionSummaryContent: {
      alignItems: 'center',
      justifyContent: 'space-between',
    },

    expansionDetail: {
      flexFlow: 'column',
      padding: '0',
      overflowX: 'auto',
    },

    expansionDetailLine: {
      display: 'inline-flex',
      justifyContent: 'space-between',
      padding: theme.spacing(0.5),
    },

    noPadding: {
      padding: 0,
    },

    liftFloorLabelStopped: {
      ...liftFloorLabelBase,
      borderColor: theme.palette.info.main,
    },

    liftFloorLabelMoving: {
      ...liftFloorLabelBase,
      borderColor: theme.palette.warning.main,
    },

    liftFloorLabelUnknown: {
      ...liftFloorLabelBase,
      borderStyle: 'none',
    },
  };
});
