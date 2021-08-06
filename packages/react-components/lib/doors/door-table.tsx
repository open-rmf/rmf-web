import React from 'react';
import * as RmfModels from 'rmf-models';
import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';

import { DoorPanelAndTableProps, DoorInfoProps, doorModeToString, doorTypeToString } from './utils';

const useStyles = makeStyles((theme) => ({
  taskRowHover: {
    background: theme.palette.action.hover,
  },
  doorLabelOpen: {
    color: theme.palette.success.main,
  },
  doorLabelClosed: {
    color: theme.palette.error.main,
  },
  doorLabelMoving: {
    color: theme.palette.warning.main,
  },
}));

const getOpMode = (doorState: RmfModels.DoorState) => {
  const getState = doorModeToString(doorState);
  return getState === 'N/A' ? 'Offline' : 'Online';
};

const TaskRow = (props: DoorInfoProps) => {
  const { door, doorState } = props;
  const classes = useStyles();
  const [hover, setHover] = React.useState(false);

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
    <>
      <TableRow
        className={hover ? classes.taskRowHover : ''}
        onMouseOver={() => setHover(true)}
        onMouseOut={() => setHover(false)}
      >
        <TableCell>{door.name}</TableCell>
        <TableCell
          className={
            getOpMode(doorState) === 'Offline' ? classes.doorLabelClosed : classes.doorLabelOpen
          }
        >
          {getOpMode(doorState)}
        </TableCell>
        <TableCell>{door.level}</TableCell>
        <TableCell>{doorTypeToString(door.door_type)}</TableCell>
        <TableCell className={doorStatusClass}>{doorModeToString(doorState)}</TableCell>
      </TableRow>
    </>
  );
};

export const DoorTable = (props: DoorPanelAndTableProps) => {
  const { doors, doorStates, onDoorControlClick } = props;

  return (
    <Table stickyHeader size="small" aria-label="door-table">
      <TableHead>
        <TableRow>
          <TableCell>Door Name</TableCell>
          <TableCell>Op. Mode</TableCell>
          <TableCell>Level</TableCell>
          <TableCell>Door Type</TableCell>
          <TableCell>Doors State</TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {doors.map((door) => (
          <TaskRow
            door={door}
            doorState={doorStates[door.name]}
            onDoorControlClick={onDoorControlClick}
          />
        ))}
      </TableBody>
    </Table>
  );
};
