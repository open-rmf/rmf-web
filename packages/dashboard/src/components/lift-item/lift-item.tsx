import {
  Accordion,
  AccordionDetails,
  AccordionProps,
  AccordionSummary,
  makeStyles,
} from '@material-ui/core';
import { ExpandMore as ExpandMoreIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { AntTab, AntTabs, TabPanel } from '../tab';
import { LiftInformation } from './lift-item-information';
import LiftRequestForm from './lift-item-form';
import { LiftRequestManager } from '../../lift-state-manager';
import OmniPanelStatusLabels from '../omni-panel-status-labels';
import { colorPalette } from '../../util/css-utils';
import Debug from 'debug';
import { CSSProperties } from '@material-ui/core/styles/withStyles';

const debug = Debug('OmniPanel:LiftItem');

export interface LiftItemProps extends Omit<AccordionProps, 'children'> {
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

export const LiftItem = React.memo(
  React.forwardRef(function (
    props: LiftItemProps,
    ref: React.Ref<HTMLElement>,
  ): React.ReactElement {
    debug('render');

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
      <Accordion ref={ref} id={id} {...otherProps}>
        <AccordionSummary
          classes={{ content: classes.accordionSummaryContent }}
          expandIcon={<ExpandMoreIcon />}
        >
          <OmniPanelStatusLabels
            modalLabelClass={liftFloorLabel(liftState)}
            name={lift.name}
            modeText={liftState ? liftState.current_floor : 'N/A'}
          />
        </AccordionSummary>
        <AccordionDetails className={classes.accordionDetail}>
          <AntTabs
            variant="fullWidth"
            value={tabValue}
            onChange={handleChange}
            aria-label="scrollable auto tabs example"
          >
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
        </AccordionDetails>
      </Accordion>
    );
  }),
);

const useStyles = makeStyles((theme) => {
  const liftFloorLabelBase: CSSProperties = {
    borderRadius: theme.shape.borderRadius,
    borderStyle: 'solid',
    border: 2,
    padding: 5,
    width: '4rem',
    textAlign: 'center',
  };

  return {
    accordionSummaryContent: {
      alignItems: 'center',
      justifyContent: 'space-between',
    },

    accordionDetail: {
      flexFlow: 'column',
      overflowX: 'auto',
      padding: 0,
    },

    accordionDetailLine: {
      display: 'inline-flex',
      justifyContent: 'space-between',
      padding: theme.spacing(0.5),
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
      borderColor: colorPalette.unknown,
    },
  };
});
