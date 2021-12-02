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
import type { Lift, LiftState } from 'api-client';
import React from 'react';
import AutoSizer from 'react-virtualized-auto-sizer';
import { FixedSizeGrid, GridChildComponentProps } from 'react-window';
import { DoorMode as RmfDoorMode } from 'rmf-models';
import LiftRequestFormDialog from './lift-request-form-dialog';
import { LiftTable } from './lift-table';
import {
  doorStateToString,
  motionStateToString,
  requestDoorModes,
  requestModes,
  onLiftClick,
} from './lift-utils';
import { LeafletContextInterface } from '@react-leaflet/core';

export interface LiftPanelProps {
  lifts: Lift[];
  liftStates: Record<string, LiftState>;
  leafletMap?: LeafletContextInterface;
  onRequestSubmit?(
    event: React.FormEvent,
    lift: Lift,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
}

interface LiftGridData extends LiftPanelProps {
  columnCount: number;
}

interface LiftGridRendererProps extends GridChildComponentProps {
  data: LiftGridData;
}

export interface LiftCellProps {
  lift: Lift;
  doorState?: number;
  motionState?: number;
  currentFloor?: string;
  leafletMap?: LeafletContextInterface;
  destinationFloor?: string;
  onRequestSubmit?(
    event: React.FormEvent,
    lift: Lift,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
}

const useStyles = makeStyles((theme) => ({
  container: {
    maxHeight: '40vh',
    margin: theme.spacing(1),
  },
  buttonBar: {
    display: 'flex',
    justifyContent: 'flex-end',
    borderRadius: 0,
    backgroundColor: theme.palette.primary.main,
  },
  grid: {
    padding: theme.spacing(2),
  },
  cellPaper: {
    padding: theme.spacing(2),
    margin: theme.spacing(1),
    backgroundColor: theme.palette.background.paper,
    border: 1,
    borderStyle: 'solid',
    borderColor: theme.palette.primary.main,
    '&:hover': {
      cursor: 'pointer',
      backgroundColor: theme.palette.action.hover,
    },
  },
  requestButton: {
    marginTop: theme.spacing(1),
    padding: theme.spacing(1),
    backgroundColor: theme.palette.info.light,
    margin: 'auto',
  },
  itemIcon: {
    color: theme.palette.primary.contrastText,
  },
  buttonGroup: {
    marginTop: theme.spacing(1),
    display: 'flex',
    justifyContent: 'center',
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
  nameField: {
    fontWeight: 'bold',
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
}));

const LiftCell = React.memo(
  ({
    lift,
    doorState,
    motionState,
    currentFloor,
    leafletMap,
    destinationFloor,
    onRequestSubmit,
  }: LiftCellProps): JSX.Element | null => {
    const labelId = `lift-cell-${lift.name}`;
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
          case RmfDoorMode.MODE_OPEN:
            return `${classes.doorLabelOpen}`;
          case RmfDoorMode.MODE_CLOSED:
            return `${classes.doorLabelClosed}`;
          case RmfDoorMode.MODE_MOVING:
            return `${classes.doorLabelMoving}`;
          default:
            return '';
        }
      },
      [classes],
    );

    return (
      <Paper
        className={classes.cellPaper}
        role="region"
        aria-labelledby={labelId}
        onClick={() => onLiftClick(lift, leafletMap)}
      >
        <Grid container direction="row">
          <Grid item xs={9}>
            <Typography
              id={labelId}
              align="center"
              className={classes.nameField}
              title={lift?.name}
            >
              {lift?.name}
            </Typography>
            <Box border={1} borderColor="divider" marginTop={1} marginBottom={1}>
              <Typography align="center">{destinationFloor || 'Unknown'}</Typography>
            </Box>
            <Typography variant="body2" align="center" className={doorModeLabelClasses(doorState)}>
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
        {lift && (
          <LiftRequestFormDialog
            lift={lift}
            availableDoorModes={requestDoorModes}
            availableRequestTypes={requestModes}
            showFormDialog={showForms}
            onRequestSubmit={onRequestSubmit}
            onClose={() => setShowForms(false)}
          />
        )}
      </Paper>
    );
  },
);

const LiftGridRenderer = ({ data, columnIndex, rowIndex, style }: LiftGridRendererProps) => {
  let lift: Lift | undefined;
  let liftState: LiftState | undefined;
  let doorState: number | undefined;
  let motionState: number | undefined;
  let destinationFloor: string | undefined;
  let currentFloor: string | undefined;
  let leafletMap: LeafletContextInterface | undefined;
  const columnCount = data.columnCount;

  if (rowIndex * columnCount + columnIndex <= data.lifts.length - 1) {
    lift = data.lifts[rowIndex * columnCount + columnIndex];
    liftState = data.liftStates[lift.name];
    doorState = liftState?.door_state;
    motionState = liftState?.motion_state;
    destinationFloor = liftState?.destination_floor;
    currentFloor = liftState?.current_floor;
    leafletMap = data.leafletMap;
  }

  return lift ? (
    <div style={style}>
      <LiftCell
        lift={lift}
        doorState={doorState}
        motionState={motionState}
        currentFloor={currentFloor}
        destinationFloor={destinationFloor}
        leafletMap={leafletMap}
        onRequestSubmit={data.onRequestSubmit}
      />
    </div>
  ) : null;
};

export function LiftPanel({
  lifts,
  liftStates,
  leafletMap,
  onRequestSubmit,
}: LiftPanelProps): JSX.Element {
  const classes = useStyles();
  const [isCellView, setIsCellView] = React.useState(true);
  const columnWidth = 250;

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
      <Grid className={classes.grid} container direction="row" spacing={2}>
        {isCellView ? (
          <AutoSizer disableHeight>
            {({ width }) => {
              const columnCount = Math.floor(width / columnWidth);
              return (
                <FixedSizeGrid
                  columnCount={columnCount}
                  columnWidth={columnWidth}
                  height={250}
                  rowCount={Math.ceil(lifts.length / columnCount)}
                  rowHeight={180}
                  width={width}
                  itemData={{
                    columnCount,
                    lifts,
                    liftStates,
                    onRequestSubmit,
                    leafletMap,
                  }}
                >
                  {LiftGridRenderer}
                </FixedSizeGrid>
              );
            }}
          </AutoSizer>
        ) : (
          <LiftTable
            leafletMap={leafletMap}
            lifts={lifts}
            liftStates={liftStates}
            onRequestSubmit={onRequestSubmit}
          />
        )}
      </Grid>
    </Card>
  );
}
