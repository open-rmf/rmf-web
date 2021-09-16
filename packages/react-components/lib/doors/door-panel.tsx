import {
  Button,
  ButtonGroup,
  Card,
  CardProps,
  Grid,
  IconButton,
  Paper,
  Typography,
  styled,
} from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { DoorTable } from './door-table';
import { DoorData, doorModeToString, doorTypeToString } from './utils';

export interface DoorPanelProps {
  doors: DoorData[];
  doorStates: Record<string, RmfModels.DoorState>;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

export interface DoorInfoProps {
  door: DoorData;
  doorState: RmfModels.DoorState;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

const classes = {
  container: 'door-panel-container',
  buttonBar: 'button-bar',
  grid: 'door-panel-grid',
  doorLabelOpen: 'door-label-open-panel',
  doorLabelClosed: 'door-label-closed-panel',
  doorLabelMoving: 'door-label-moving-panel',
  cellPaper: 'door-panel-cell-paper',
  itemIcon: 'door-panel-item-icon',
  buttonGroup: 'door-panel-button-group',
  panelHeader: 'door-panel-panel-header',
};
const DoorPanelRoot = styled((props: CardProps) => <Card {...props} />)(({ theme }) => ({
  [`& .${classes.container}`]: {
    margin: theme.spacing(1),
  },
  [`& .${classes.buttonBar}`]: {
    display: 'flex',
    justifyContent: 'flex-end',
    borderRadius: '0px',
    backgroundColor: theme.palette.primary.main,
  },
  [`& .${classes.grid}`]: {
    padding: '1rem',
  },
  [`& .${classes.doorLabelOpen}`]: {
    backgroundColor: theme.palette.success.main,
  },
  [`& .${classes.doorLabelClosed}`]: {
    backgroundColor: theme.palette.error.main,
  },
  [`& .${classes.doorLabelMoving}`]: {
    backgroundColor: theme.palette.warning.main,
  },
  [`& .${classes.cellPaper}`]: {
    padding: '0.5rem',
    backgroundColor: theme.palette.info.light,
  },
  [`& .${classes.itemIcon}`]: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
  },
  [`& .${classes.buttonGroup}`]: {
    display: 'flex',
    justifyContent: 'center',
  },
  [`& .${classes.panelHeader}`]: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
    marginLeft: '1rem',
  },
}));

const DoorCell = React.memo(
  ({ door, doorState, onDoorControlClick }: DoorInfoProps): JSX.Element => {
    const doorModeLabelClasses = React.useCallback(
      (doorState?: RmfModels.DoorState): string => {
        if (!doorState) {
          return '';
        }
        switch (doorState.current_mode.value) {
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

    const doorStatusClass = doorModeLabelClasses(doorState);
    const labelId = `door-cell-${door.door.name}`;

    return (
      <Paper className={classes.cellPaper} role="region" aria-labelledby={labelId}>
        <Typography id={labelId} variant="body1" align="center" style={{ fontWeight: 'bold' }}>
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
              {doorModeToString(doorState)}
            </Typography>
          </Grid>
        </Grid>
        <Typography variant="body1" align="center">
          {doorTypeToString(door.door.door_type)}
        </Typography>
        <div className={classes.buttonGroup}>
          <ButtonGroup size="small">
            <Button
              onClick={(ev) =>
                onDoorControlClick &&
                onDoorControlClick(ev, door.door, RmfModels.DoorMode.MODE_OPEN)
              }
            >
              Open
            </Button>
            <Button
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

export function DoorPanel({ doors, doorStates, onDoorControlClick }: DoorPanelProps): JSX.Element {
  const [isCellView, setIsCellView] = React.useState(true);

  return (
    <DoorPanelRoot variant="outlined" className={classes.container}>
      <Paper className={classes.buttonBar}>
        <Grid container direction="row" justifyContent="space-between" alignItems="center">
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
          doors.map((door) => {
            return (
              <Grid item xs={4} key={door.door.name}>
                <DoorCell
                  door={door}
                  doorState={doorStates[door.door.name]}
                  onDoorControlClick={onDoorControlClick}
                />
              </Grid>
            );
          })
        ) : (
          <DoorTable
            doors={doors}
            doorStates={doorStates}
            onDoorControlClick={onDoorControlClick}
          />
        )}
      </Grid>
    </DoorPanelRoot>
  );
}
