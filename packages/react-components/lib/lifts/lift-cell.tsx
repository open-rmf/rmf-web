import { Box, Button, Grid, makeStyles, Paper, Typography } from '@material-ui/core';
import ArrowDownwardIcon from '@material-ui/icons/ArrowDownward';
import ArrowUpwardIcon from '@material-ui/icons/ArrowUpward';
import React from 'react';
import * as RmfModels from 'rmf-models';
import LiftRequestFormDialog from './lift-request-form-dialog';
import {
  doorStateToString,
  motionStateToString,
  requestDoorModes,
  requestModes,
} from './lift-utils';

export interface LiftCellProps {
  lift: RmfModels.Lift;
  doorState?: number;
  motionState?: number;
  destinationFloor?: string;
  currentFloor?: string;
  onRequestSubmit?(
    event: React.FormEvent,
    lift: RmfModels.Lift,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
}

const useStyles = makeStyles((theme) => ({
  container: {
    margin: theme.spacing(1),
  },
  buttonBar: {
    display: 'flex',
    justifyContent: 'flex-end',
    borderRadius: '0px',
    backgroundColor: theme.palette.primary.main,
  },
  cellPaper: {
    padding: '0.5rem',
    backgroundColor: theme.palette.info.light,
  },
  buttonGroup: {
    display: 'flex',
    justifyContent: 'center',
  },
  iconMoving: {
    color: theme.palette.success.dark,
  },
  iconOtherStates: {
    color: 'white',
  },
  doorLabelOpen: {
    backgroundColor: theme.palette.success.main,
  },
  doorLabelClosed: {
    backgroundColor: theme.palette.error.main,
  },
  doorLabelMoving: {
    backgroundColor: theme.palette.warning.main,
  },
  panelHeader: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
    marginLeft: '1rem',
  },
}));

export const LiftCell: React.FC<LiftCellProps> = ({
  lift,
  doorState,
  motionState,
  destinationFloor,
  currentFloor,
  onRequestSubmit,
}: LiftCellProps): JSX.Element => {
  const classes = useStyles();

  const [showForms, setShowForms] = React.useState(false);

  const currMotion = motionStateToString(motionState);
  const getMotionArrowColor = (currMotion: string, arrowDirection: string) => {
    return currMotion === arrowDirection ? classes.iconMoving : classes.iconOtherStates;
  };

  const currDoorMotion = doorStateToString(doorState);
  const doorModeLabelClasses = React.useCallback(
    (doorState?: number): string => {
      switch (doorState) {
        case RmfModels.DoorMode.MODE_OPEN:
          return `${classes.doorLabelOpen}`;
        case RmfModels.DoorMode.MODE_CLOSED:
          return `${classes.doorLabelClosed}`;
        case RmfModels.DoorMode.MODE_MOVING:
          return `${classes.doorLabelMoving}`;
        default:
          return '';
      }
    },
    [classes],
  );

  const labelId = `lift-cell-${lift.name}`;

  return (
    <Paper className={classes.cellPaper} role="region" aria-labelledby={labelId}>
      <Grid container direction="row">
        <Grid item xs={9}>
          <Typography id={labelId} align="center" style={{ fontWeight: 'bold' }}>
            {lift.name}
          </Typography>
          <Box border={1} borderColor="divider" m={0.5}>
            <Typography align="center">{destinationFloor || 'Unknown'}</Typography>
          </Box>
          <Typography align="center" className={doorModeLabelClasses(doorState)}>
            {currDoorMotion}
          </Typography>
        </Grid>
        <Grid item xs>
          <Typography align="center" className={getMotionArrowColor(currMotion, 'Up')}>
            <ArrowUpwardIcon />
          </Typography>
          <Typography align="center">{currentFloor || '?'}</Typography>
          <Typography align="center" className={getMotionArrowColor(currMotion, 'Down')}>
            <ArrowDownwardIcon />
          </Typography>
        </Grid>
      </Grid>
      <Button
        variant="contained"
        color="primary"
        fullWidth
        size="small"
        onClick={() => setShowForms(true)}
      >
        Request Form
      </Button>
      <LiftRequestFormDialog
        lift={lift}
        availableDoorModes={requestDoorModes}
        availableRequestTypes={requestModes}
        showFormDialog={showForms}
        onRequestSubmit={onRequestSubmit}
        onClose={() => setShowForms(false)}
      />
    </Paper>
  );
};
