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
import type { Door, DoorState } from 'api-client';
import React from 'react';
import clsx from 'clsx';
import AutoSizer from 'react-virtualized-auto-sizer';
import { FixedSizeGrid, GridChildComponentProps } from 'react-window';
import { DoorMode as RmfDoorMode } from 'rmf-models';
import { DoorTable } from './door-table';
import { DoorData, doorModeToString, doorTypeToString, onDoorClick } from './utils';
import { LeafletContextInterface } from '@react-leaflet/core';

export interface DoorPanelProps {
  doors: DoorData[];
  doorStates: Record<string, DoorState>;
  leafletMap?: LeafletContextInterface;
  onDoorControlClick?(event: React.MouseEvent, door: Door, mode: number): void;
}

interface DoorGridData extends DoorPanelProps {
  columnCount: number;
}

interface DoorGridRendererProps extends GridChildComponentProps {
  data: DoorGridData;
}

export interface DoorcellProps {
  door: DoorData;
  doorMode?: number;
  leafletMap?: LeafletContextInterface;
  onDoorControlClick?(event: React.MouseEvent, door: Door, mode: number): void;
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
    padding: theme.spacing(1),
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
  itemIcon: {
    color: theme.palette.primary.contrastText,
  },
  buttonGroup: {
    marginTop: theme.spacing(1),
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
  nameField: {
    fontWeight: 'bold',
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
}));

const DoorCell = React.memo(
  ({ door, doorMode, leafletMap, onDoorControlClick }: DoorcellProps): JSX.Element => {
    const classes = useStyles();
    const doorModeLabelClasses = React.useCallback(
      (doorMode?: number): string => {
        if (doorMode === undefined) {
          return '';
        }
        switch (doorMode) {
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
          className={classes.nameField}
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
          {door && doorTypeToString(door.door.door_type)}
        </Typography>
        <div className={classes.buttonGroup}>
          <ButtonGroup variant="contained" size="small">
            <Button
              className={classes.openCloseButtons}
              onClick={(ev) =>
                onDoorControlClick && onDoorControlClick(ev, door.door, RmfDoorMode.MODE_OPEN)
              }
            >
              Open
            </Button>
            <Button
              className={classes.openCloseButtons}
              onClick={(ev) =>
                onDoorControlClick && onDoorControlClick(ev, door.door, RmfDoorMode.MODE_CLOSED)
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
  let leafletMap: Map | undefined;
  let doorState: DoorState | undefined;
  const columnCount = data.columnCount;

  if (rowIndex * columnCount + columnIndex <= data.doors.length - 1) {
    door = data.doors[rowIndex * columnCount + columnIndex];
    doorState = data.doorStates[door.door.name];
    leafletMap = data.leafletMap;
  }

  return door ? (
    <div style={style}>
      <DoorCell
        door={door}
        doorMode={doorState?.current_mode.value}
        onDoorControlClick={data.onDoorControlClick}
        leafletMap={leafletMap}
      />
    </div>
  ) : null;
};

export function DoorPanel({
  doors,
  doorStates,
  leafletMap,
  onDoorControlClick,
}: DoorPanelProps): JSX.Element {
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
                  rowCount={Math.ceil(doors.length / columnCount)}
                  rowHeight={150}
                  width={width}
                  itemData={{
                    columnCount,
                    doors,
                    doorStates,
                    onDoorControlClick,
                    leafletMap,
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
            leafletMap={leafletMap}
          />
        )}
      </Grid>
    </Card>
  );
}
