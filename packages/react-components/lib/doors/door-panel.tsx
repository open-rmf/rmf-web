import React from 'react';
import * as RmfModels from 'rmf-models';
import {
  Paper,
  IconButton,
  makeStyles,
  Grid,
  Typography,
  Button,
  ButtonGroup,
} from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';

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

const useStyles = makeStyles((theme) => ({
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
  },
  itemIcon: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
  },
  buttonGroup: {
    display: 'flex',
    justifyContent: 'center',
  },
}));

const DoorCell = (props: DoorInfoProps): JSX.Element => {
  const { door, doorState, onDoorControlClick } = props;
  const classes = useStyles();

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

  return (
    <Paper className={classes.cellPaper} data-item={door.door.name}>
      <Typography variant="body1" align="center">
        {door.door.name}
      </Typography>
      <Grid container direction="row" spacing={1}>
        <Grid item xs={6}>
          <Typography variant="body2" align="center">
            {door.level}
          </Typography>
        </Grid>
        <Grid item xs={6}>
          <Typography className={doorStatusClass} variant="body2" align="center">
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
            aria-label={`${door.door.name}_open`}
            onClick={(ev) =>
              onDoorControlClick && onDoorControlClick(ev, door.door, RmfModels.DoorMode.MODE_OPEN)
            }
          >
            Open
          </Button>
          <Button
            aria-label={`${door.door.name}_close`}
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
};

export function DoorPanel(props: DoorPanelProps) {
  const { doors, doorStates, onDoorControlClick } = props;
  const classes = useStyles();

  const [isCellView, setIsCellView] = React.useState(true);

  return (
    <div>
      <Paper className={classes.buttonBar}>
        <IconButton
          aria-label="view-mode"
          className={classes.itemIcon}
          onClick={() => setIsCellView(!isCellView)}
        >
          {isCellView ? <ViewListIcon /> : <ViewModuleIcon />}
        </IconButton>
      </Paper>
      <Grid className={classes.grid} container direction="row" spacing={1}>
        {isCellView ? (
          doors.map((door, i) => {
            return (
              <Grid item xs={4} key={`${door.door.name}_${i}`}>
                <DoorCell
                  door={door}
                  doorState={doorStates[door.door.name]}
                  onDoorControlClick={onDoorControlClick}
                />
              </Grid>
            );
          })
        ) : (
          <DoorTable doors={doors} doorStates={doorStates} />
        )}
      </Grid>
    </div>
  );
}
