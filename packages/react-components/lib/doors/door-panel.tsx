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
import * as RmfModels from 'rmf-models';
import { DoorTable } from './door-table';
import { DoorData, doorModeToString, doorTypeToString } from './utils';
import { FixedSizeGrid, GridChildComponentProps } from 'react-window';
import AutoSizer from 'react-virtualized-auto-sizer';

export interface DoorPanelProps {
  doors: DoorData[];
  doorStates: Record<string, RmfModels.DoorState>;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

export interface DoorGridDataProps extends DoorPanelProps {
  columnCount: number;
}

export interface DoorGridRendererProps extends GridChildComponentProps {
  data: DoorGridDataProps;
}

export interface DoorcellProps {
  door: DoorData;
  doorState?: RmfModels.DoorState;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
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
  grid: {
    padding: '1rem',
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
  cellPaper: {
    padding: '0.5rem',
    backgroundColor: theme.palette.info.light,
    margin: '0.5rem',
  },
  itemIcon: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
  },
  buttonGroup: {
    display: 'flex',
    justifyContent: 'center',
  },
  panelHeader: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
    marginLeft: '1rem',
  },
  nameField: {
    fontWeight: 'bold',
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
}));

const DoorCell = React.memo(
  ({ door, doorState, onDoorControlClick }: DoorcellProps): JSX.Element => {
    const classes = useStyles();
    const doorMode = doorState?.current_mode.value;
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
      <Paper className={classes.cellPaper} role="region" aria-labelledby={labelId}>
        <Typography
          id={labelId}
          variant="body1"
          align="center"
          className={classes.nameField}
          title={door.door.name}
        >
          {door.door.name}
        </Typography>
        <Grid container direction="row" spacing={1}>
          <Grid item xs={6}>
            <Typography variant="body2" align="center">
              {door.level}
            </Typography>
          </Grid>
          <Grid item xs={6}>
            <Typography className={doorStatusClass} variant="body2" align="center" role="status">
              {doorModeToString(doorMode)}
            </Typography>
          </Grid>
        </Grid>
        <Typography variant="body1" align="center">
          {door && doorTypeToString(door.door.door_type)}
        </Typography>
        <div className={classes.buttonGroup}>
          <ButtonGroup size="small">
            <Button
              onClick={(ev) =>
                door &&
                onDoorControlClick &&
                onDoorControlClick(ev, door.door, RmfModels.DoorMode.MODE_OPEN)
              }
            >
              Open
            </Button>
            <Button
              onClick={(ev) =>
                door &&
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

const DoorGridRenderer = ({ data, columnIndex, rowIndex, style }: DoorGridRendererProps) => {
  let door: DoorData | undefined;
  let doorState: RmfModels.DoorState | undefined;
  const columnCount = data.columnCount;

  if (rowIndex * columnCount + columnIndex <= data.doors.length - 1) {
    door = data.doors[rowIndex * columnCount + columnIndex];
    doorState = data.doorStates[door.door.name];
  }

  return door ? (
    <div style={style}>
      <DoorCell door={door} doorState={doorState} onDoorControlClick={data.onDoorControlClick} />
    </div>
  ) : null;
};

export function DoorPanel({ doors, doorStates, onDoorControlClick }: DoorPanelProps): JSX.Element {
  const classes = useStyles();
  const [isCellView, setIsCellView] = React.useState(true);
  const columnWidth = 250;

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
      <Grid className={classes.grid} container direction="row" spacing={1}>
        {isCellView ? (
          <AutoSizer disableHeight>
            {({ width }) => {
              const columnCount = Math.floor(width / columnWidth);
              return (
                <FixedSizeGrid
                  columnCount={columnCount}
                  columnWidth={columnWidth}
                  height={250}
                  rowCount={Math.ceil(doors.length / columnCount)}
                  rowHeight={120}
                  width={width}
                  itemData={{
                    columnCount,
                    doors,
                    doorStates,
                    onDoorControlClick,
                  }}
                >
                  {DoorGridRenderer}
                </FixedSizeGrid>
              );
            }}
          </AutoSizer>
        ) : (
          <DoorTable
            doors={doors}
            doorStates={doorStates}
            onDoorControlClick={onDoorControlClick}
          />
        )}
      </Grid>
    </Card>
  );
}
