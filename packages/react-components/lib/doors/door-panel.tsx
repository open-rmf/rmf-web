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

export interface DoorPanelProps {
  doors: DoorData[];
  doorStates: Record<string, RmfModels.DoorState>;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

export interface DoorInfoProps extends GridChildComponentProps {
  data: DoorPanelProps;
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
}));

const DoorCell = React.memo(
  ({ data, columnIndex, rowIndex, style }: DoorInfoProps): JSX.Element | null => {
    const classes = useStyles();
    const columnCount = 3;
    let door: DoorData | undefined;
    let doorMode: number | undefined;
    let doorStatusClass: string | undefined;
    let labelId: string | undefined;

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

    if (rowIndex * columnCount + columnIndex <= data.doors.length - 1) {
      door = data.doors[rowIndex * columnCount + columnIndex];
      doorMode = data.doorStates[door.door.name]?.current_mode.value;
      doorStatusClass = doorModeLabelClasses(doorMode);
      labelId = `door-cell-${door.door.name}`;
    }
    const onDoorControlClick = data.onDoorControlClick;
    return door ? (
      <div style={style}>
        <Paper className={classes.cellPaper} role="region" aria-labelledby={labelId}>
          <Typography id={labelId} variant="body1" align="center" style={{ fontWeight: 'bold' }}>
            {door?.door.name}
          </Typography>
          <Grid container direction="row" spacing={1}>
            <Grid item xs={6}>
              <Typography variant="body2" align="center">
                {door?.level}
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
      </div>
    ) : null;
  },
);

export function DoorPanel({ doors, doorStates, onDoorControlClick }: DoorPanelProps): JSX.Element {
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
      <Grid className={classes.grid} container direction="row" spacing={1}>
        {isCellView ? (
          <FixedSizeGrid
            columnCount={3}
            columnWidth={250}
            height={250}
            rowCount={Math.ceil(doors.length / 3)}
            rowHeight={120}
            width={760}
            itemData={{
              doors,
              doorStates,
              onDoorControlClick,
            }}
          >
            {DoorCell}
          </FixedSizeGrid>
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
