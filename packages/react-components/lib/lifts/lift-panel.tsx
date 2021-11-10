import {
  Box,
  Button,
  Card,
  Grid,
  IconButton,
  makeStyles,
  Paper,
  Typography,
} from '@material-ui/core';
import ArrowDownwardIcon from '@material-ui/icons/ArrowDownward';
import ArrowUpwardIcon from '@material-ui/icons/ArrowUpward';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';
import React from 'react';
import * as RmfModels from 'rmf-models';
import LiftRequestFormDialog from './lift-request-form-dialog';
import { LiftTable } from './lift-table';
import {
  doorStateToString,
  motionStateToString,
  requestDoorModes,
  requestModes,
} from './lift-utils';

export interface LiftPanelProps {
  lifts: RmfModels.Lift[];
  liftStates: Record<string, RmfModels.LiftState>;
  onRequestSubmit?(
    event: React.FormEvent,
    lift: RmfModels.Lift,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
}

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
    backgroundColor: theme.palette.background.default,
  },
  buttonBar: {
    display: 'flex',
    justifyContent: 'flex-end',
    borderRadius: 0,
    backgroundColor: theme.palette.primary.main,
  },
  grid: {
    padding: theme.spacing(1),
  },
  cellPaper: {
    padding: theme.spacing(2),
    backgroundColor: theme.palette.background.paper,
    margin: theme.spacing(1),
  },
  requestButton: {
    marginTop: theme.spacing(1),
  },
  itemIcon: {
    color: theme.palette.primary.contrastText,
  },
  iconMoving: {
    color: theme.palette.success.main,
  },
  iconOtherStates: {
    color: theme.palette.primary.main,
  },
  doorLabelOpen: {
    backgroundColor: theme.palette.success.main,
    color: theme.palette.success.contrastText,
  },
  doorLabelClosed: {
    backgroundColor: theme.palette.error.main,
    color: theme.palette.error.contrastText,
  },
  doorLabelMoving: {
    backgroundColor: theme.palette.warning.main,
    color: theme.palette.warning.contrastText,
  },
  panelHeader: {
    color: theme.palette.primary.contrastText,
    marginLeft: theme.spacing(2),
  },
}));

const LiftCell = React.memo(
  ({
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
            <Box border={1} borderColor="divider" marginTop={1} marginBottom={1}>
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
          className={classes.requestButton}
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
  },
);

export function LiftPanel({ lifts, liftStates, onRequestSubmit }: LiftPanelProps): JSX.Element {
  const classes = useStyles();
  const [isCellView, setIsCellView] = React.useState(true);

  return (
    <Card variant="outlined" className={classes.container}>
      <Paper className={classes.buttonBar}>
        <Grid container direction="row" justify="space-between" alignItems="center">
          <Grid item xs={6}>
            <Typography variant="h5" className={classes.panelHeader}>
              Lifts
            </Typography>
          </Grid>
          <Grid item>
            <IconButton
              aria-label="view mode"
              className={classes.itemIcon}
              onClick={() => setIsCellView(!isCellView)}
            >
              {isCellView ? <ViewListIcon /> : <ViewModuleIcon />}
            </IconButton>
          </Grid>
        </Grid>
      </Paper>
      <Grid className={classes.grid} container direction="row" spacing={1}>
        {isCellView ? (
          lifts.map((lift, i) => {
            const state: RmfModels.LiftState | undefined = liftStates[lift.name];
            return (
              <Grid item xs={4} key={`${lift.name}_${i}`}>
                <LiftCell
                  lift={lift}
                  doorState={state?.door_state}
                  motionState={state?.motion_state}
                  destinationFloor={state?.destination_floor}
                  currentFloor={state?.current_floor}
                  onRequestSubmit={onRequestSubmit}
                />
              </Grid>
            );
          })
        ) : (
          <LiftTable lifts={lifts} liftStates={liftStates} onRequestSubmit={onRequestSubmit} />
        )}
      </Grid>
    </Card>
  );
}
