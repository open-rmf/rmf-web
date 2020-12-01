import { Accordion, AccordionProps, makeStyles, Divider } from '@material-ui/core';
import Typography from '@material-ui/core/Typography';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import ItemAccordionDetails from '../item-accordion-details';
import ItemAccordionSummary from '../item-accordion-summary';
import { SimpleInfo } from '../simple-info';

const debug = Debug('Dispensers:DispenserAccordion');

interface DispenserInfoProps {
  dispenser: RomiCore.DispenserState;
}

const DispenserInfo = (props: DispenserInfoProps) => {
  const { dispenser } = props;

  const data = [
    { name: 'Name', value: dispenser.guid },
    { name: 'No. Queued Requests', value: dispenser.request_guid_queue.length },
    {
      name: 'Request Queue ID',
      value: dispenser.request_guid_queue.length ? dispenser.request_guid_queue : 'Unknown',
      disabled: !!dispenser.request_guid_queue.length,
    },
    { name: 'Seconds Remaining', value: dispenser.seconds_remaining },
  ];

  return <SimpleInfo infoData={data} />;
};

function dispenserModeToString(mode: number): string {
  switch (mode) {
    case RomiCore.DispenserState.IDLE:
      return 'IDLE';
    case RomiCore.DispenserState.BUSY:
      return 'ONLINE';
    case RomiCore.DispenserState.OFFLINE:
      return 'OFFLINE';
    default:
      return 'N/A';
  }
}

const useStyles = makeStyles((theme) => ({
  statusLabelIdle: { borderColor: theme.palette.warning.main },
  statusLabelBusy: { borderColor: theme.palette.success.main },
  statusLabelOffline: { borderColor: theme.palette.error.main },
  unknownStateDesc: {
    padding: '0.5rem 0',
  },
}));

export interface DispenserAccordionProps extends Omit<AccordionProps, 'children'> {
  dispenserState: RomiCore.DispenserState | null;
  dispensers: string;
}

export const DispenserAccordion = React.forwardRef(
  (props: DispenserAccordionProps, ref: React.Ref<HTMLElement>) => {
    const { dispenserState, dispensers, ...otherProps } = props;
    debug(`render ${dispenserState?.guid}`);
    const classes = useStyles();

    const getStatusLabelClass = () => {
      switch (dispenserState?.mode) {
        case RomiCore.DispenserState.IDLE:
          return classes.statusLabelIdle;
        case RomiCore.DispenserState.BUSY:
          return classes.statusLabelBusy;
        case RomiCore.DispenserState.OFFLINE:
          return classes.statusLabelOffline;
        default:
          return null;
      }
    };

    const statusLabelClass = getStatusLabelClass();

    return (
      <Accordion ref={ref} {...otherProps}>
        <ItemAccordionSummary
          title={dispenserState ? dispenserState.guid : dispensers}
          statusProps={{
            className: statusLabelClass ? statusLabelClass : undefined,
            text: dispenserState ? dispenserModeToString(dispenserState.mode) : 'UNKNOWN',
            variant: statusLabelClass ? 'normal' : 'unknown',
          }}
        />
        {dispenserState ? (
          <ItemAccordionDetails>
            <DispenserInfo dispenser={dispenserState} />
          </ItemAccordionDetails>
        ) : (
          <React.Fragment>
            <Divider />
            <Typography className={classes.unknownStateDesc} align="center" variant="body1">
              State of dispenser is unknown
            </Typography>
          </React.Fragment>
        )}
      </Accordion>
    );
  },
);

export default DispenserAccordion;
