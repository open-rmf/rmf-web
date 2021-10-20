import { Button, ButtonGroup, Grid, makeStyles, Paper, Typography } from '@material-ui/core';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { DoorData, doorModeToString, doorTypeToString } from './utils';

export interface DoorCellProps {
  door: DoorData;
  doorMode?: number;
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
  buttonGroup: {
    display: 'flex',
    justifyContent: 'center',
  },
  panelHeader: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
    marginLeft: '1rem',
  },
}));

export const DoorCell: React.FC<DoorCellProps> = ({
  door,
  doorMode,
  onDoorControlClick,
}: DoorCellProps): JSX.Element => {
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
            onClick={(ev) =>
              onDoorControlClick && onDoorControlClick(ev, door.door, RmfModels.DoorMode.MODE_OPEN)
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
};
