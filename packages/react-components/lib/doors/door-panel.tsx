import {
  Button,
  ButtonGroup,
  Card,
  Grid,
  IconButton,
  makeStyles,
  Paper,
  Typography,
} from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';
import React from 'react';
import { LeafletContext } from 'react-leaflet';
import * as RmfModels from 'rmf-models';
import { DoorTable } from './door-table';
import { DoorData, doorModeToString, doorTypeToString, onDoorClick } from './utils';
import clsx from 'clsx';

export interface DoorPanelProps {
  doors: DoorData[];
  doorStates: Record<string, RmfModels.DoorState>;
  leafletMap?: LeafletContext;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

export interface DoorInfoProps {
  door: DoorData;
  doorMode?: number;
  leafletMap?: LeafletContext;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

const useStyles = makeStyles((theme) => ({
  container: {
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
    maxHeight: '40vh',
    overflowY: 'auto',
  },
  doorLabelDefault: {
    padding: theme.spacing(0, 0.2),
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
  cellPaper: {
    padding: theme.spacing(2),
    backgroundColor: theme.palette.background.paper,
    border: 1,
    borderStyle: 'solid',
    borderColor: theme.palette.primary.main,
    '&:hover': {
      cursor: 'pointer',
      backgroundColor: theme.palette.action.hover,
    },
  },
  itemIcon: {
    color: theme.palette.primary.contrastText,
  },
  buttonGroup: {
    display: 'flex',
    justifyContent: 'center',
  },
  openCloseButtons: {
    border: 1,
    borderStyle: 'solid',
    borderColor: theme.palette.common.white,
    borderRadius: theme.shape.borderRadius,
    backgroundColor: theme.palette.primary.light,
    boxShadow: theme.shadows[3],
    color: theme.palette.primary.contrastText,
    '&:hover': {
      color: theme.palette.primary.main,
    },
  },
  panelHeader: {
    color: theme.palette.primary.contrastText,
    marginLeft: theme.spacing(2),
  },
}));

const DoorCell = React.memo(
  ({ door, doorMode, leafletMap, onDoorControlClick }: DoorInfoProps): JSX.Element => {
    const classes = useStyles();

    const doorModeLabelClasses = React.useCallback(
      (doorMode?: number): string => {
        if (doorMode === undefined) {
          return '';
        }
        switch (doorMode) {
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

    const doorStatusClass = doorModeLabelClasses(doorMode);
    const labelId = `door-cell-${door.door.name}`;

    return (
      <Paper
        className={classes.cellPaper}
        role="region"
        aria-labelledby={labelId}
        onClick={() => onDoorClick(door.door, leafletMap)}
      >
        <Typography
          noWrap
          id={labelId}
          variant="body1"
          align="center"
          style={{ fontWeight: 'bold', width: '120px' }}
          title={door.door.name}
        >
          {door.door.name}
        </Typography>
        <Grid container direction="row" spacing={1}>
          <Grid item xs={4}>
            <Typography variant="body2" align="center">
              {door.level}
            </Typography>
          </Grid>
          <Grid item xs={8}>
            <Typography
              className={clsx(doorStatusClass, classes.doorLabelDefault)}
              variant="body2"
              align="center"
              role="status"
            >
              {doorModeToString(doorMode)}
            </Typography>
          </Grid>
        </Grid>
        <Typography variant="body1" align="center">
          {doorTypeToString(door.door.door_type)}
        </Typography>
        <div className={classes.buttonGroup}>
          <ButtonGroup size="small">
            <Button
              className={classes.openCloseButtons}
              onClick={(ev) =>
                onDoorControlClick &&
                onDoorControlClick(ev, door.door, RmfModels.DoorMode.MODE_OPEN)
              }
            >
              Open
            </Button>
            <Button
              className={classes.openCloseButtons}
              onClick={(ev) =>
                onDoorControlClick &&
                onDoorControlClick(ev, door.door, RmfModels.DoorMode.MODE_CLOSED)
              }
            >
              Close
            </Button>
          </ButtonGroup>
        </div>
      </Paper>
    );
  },
);

export function DoorPanel({
  doors,
  doorStates,
  leafletMap,
  onDoorControlClick,
}: DoorPanelProps): JSX.Element {
  const classes = useStyles();

  const [isCellView, setIsCellView] = React.useState(true);

  return (
    <Card variant="outlined" className={classes.container}>
      <Paper className={classes.buttonBar}>
        <Grid container direction="row" justify="space-between" alignItems="center">
          <Grid item xs={6}>
            <Typography variant="h5" className={classes.panelHeader}>
              Doors
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
          doors.map((door) => {
            return (
              <Grid item xs="auto" key={door.door.name}>
                <DoorCell
                  door={door}
                  doorMode={doorStates[door.door.name]?.current_mode.value}
                  onDoorControlClick={onDoorControlClick}
                  leafletMap={leafletMap}
                />
              </Grid>
            );
          })
        ) : (
          <DoorTable
            doors={doors}
            doorStates={doorStates}
            onDoorControlClick={onDoorControlClick}
            leafletMap={leafletMap}
          />
        )}
      </Grid>
    </Card>
  );
}
