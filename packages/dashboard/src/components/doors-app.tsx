import { Button, SxProps, useTheme } from '@mui/material';
import { BuildingMap, DoorState } from 'api-client';
import React from 'react';
import {
  DoorCardProps as BaseDoorCardProps,
  doorModeToString,
  doorTypeToString,
} from 'react-components';
import { DoorMode, DoorMode as RmfDoorMode } from 'rmf-models';
import { createMicroApp } from './micro-app';
import { RmfAppContext } from './rmf-app';
import Table from '@mui/material/Table';
import TableBody from '@mui/material/TableBody';
import TableCell from '@mui/material/TableCell';
import TableHead from '@mui/material/TableHead';
import TableRow from '@mui/material/TableRow';

type DoorCardProps = Omit<BaseDoorCardProps, 'mode'>;

const DoorCard = ({ children, ...otherProps }: DoorCardProps) => {
  const theme = useTheme();
  const rmf = React.useContext(RmfAppContext);
  const [doorState, setDoorState] = React.useState<DoorState | null>(null);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.getDoorStateObs(otherProps.name).subscribe(setDoorState);
    return () => sub.unsubscribe();
  }, [rmf, otherProps.name]);

  const labelStyle = React.useMemo<SxProps>(() => {
    const { current_mode } = doorState ?? {};
    const mode = current_mode?.value;

    const disabled = {
      backgroundColor: theme.palette.action.disabledBackground,
      color: theme.palette.action.disabled,
    };
    const open = {
      backgroundColor: theme.palette.success.main,
      color: theme.palette.success.contrastText,
    };
    const closed = {
      backgroundColor: theme.palette.error.main,
      color: theme.palette.error.contrastText,
    };
    const moving = {
      backgroundColor: theme.palette.warning.main,
      color: theme.palette.warning.contrastText,
    };

    switch (mode) {
      case DoorMode.MODE_OPEN:
        return open;
      case DoorMode.MODE_CLOSED:
        return closed;
      case DoorMode.MODE_MOVING:
        return moving;
      default:
        return disabled;
    }
  }, [theme, doorState]);

  return (
    <TableRow key={otherProps.name} sx={{ '&:last-child td, &:last-child th': { border: 0 } }}>
      <TableCell>{otherProps.name}</TableCell>
      <TableCell>{otherProps.level}</TableCell>
      <TableCell>{doorTypeToString(otherProps.type)}</TableCell>
      <TableCell sx={labelStyle}>
        {doorState ? doorModeToString(doorState.current_mode.value) : -1}
      </TableCell>
      <TableCell align="right">
        <Button
          variant="contained"
          size="small"
          onClick={() =>
            rmf?.doorsApi.postDoorRequestDoorsDoorNameRequestPost(otherProps.name, {
              mode: RmfDoorMode.MODE_OPEN,
            })
          }
        >
          Open
        </Button>
      </TableCell>
      <TableCell align="right">
        <Button
          variant="contained"
          size="small"
          onClick={() =>
            rmf?.doorsApi.postDoorRequestDoorsDoorNameRequestPost(otherProps.name, {
              mode: RmfDoorMode.MODE_CLOSED,
            })
          }
        >
          Close
        </Button>
      </TableCell>
    </TableRow>
  );
};

export const DoorsApp = createMicroApp('Doors', () => {
  const rmf = React.useContext(RmfAppContext);
  const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.buildingMapObs.subscribe(setBuildingMap);
    return () => sub.unsubscribe();
  }, [rmf]);

  return (
    <Table stickyHeader size="small" aria-label="door-table">
      <TableHead>
        <TableRow>
          <TableCell>Name</TableCell>
          <TableCell>Level</TableCell>
          <TableCell>Type</TableCell>
          <TableCell>Door State</TableCell>
          <TableCell></TableCell>
          <TableCell></TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {buildingMap &&
          buildingMap.levels.flatMap((level) =>
            level.doors.map((door) => (
              <DoorCard
                key={door.name}
                name={door.name}
                level={level.name}
                type={door.door_type}
                sx={{ width: 200 }}
                aria-labelledby={`door-cell-${door.name}`}
              ></DoorCard>
            )),
          )}
      </TableBody>
    </Table>
  );
});
