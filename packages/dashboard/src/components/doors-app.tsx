import { Button, SxProps, Typography, useTheme } from '@mui/material';
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
      color: theme.palette.action.disabledBackground,
    };
    const open = {
      color: theme.palette.success.main,
    };
    const closed = {
      color: theme.palette.error.main,
    };
    const moving = {
      color: theme.palette.warning.main,
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
    <TableRow key={otherProps.name}>
      <TableCell>{otherProps.name}</TableCell>
      <TableCell>{otherProps.level}</TableCell>
      <TableCell>{doorTypeToString(otherProps.type)}</TableCell>
      <TableCell sx={labelStyle}>
        <Typography
          data-testid="door-state"
          component="p"
          sx={{
            fontWeight: 'bold',
            fontSize: 14,
          }}
        >
          {doorState ? doorModeToString(doorState.current_mode.value) : -1}
        </Typography>
      </TableCell>
      <TableCell align="right" sx={{ margin: 0, padding: 0, paddingRight: 1 }}>
        <Button
          variant="contained"
          size="small"
          aria-label="open"
          sx={{ marginRight: 2 }}
          onClick={() =>
            rmf?.doorsApi.postDoorRequestDoorsDoorNameRequestPost(otherProps.name, {
              mode: RmfDoorMode.MODE_OPEN,
            })
          }
        >
          Open
        </Button>
        <Button
          variant="contained"
          size="small"
          aria-label="close"
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
    <Table stickyHeader size="small" aria-label="door-table" style={{ tableLayout: 'fixed' }}>
      <TableHead>
        <TableRow>
          <TableCell>Name</TableCell>
          <TableCell>Level</TableCell>
          <TableCell>Type</TableCell>
          <TableCell>Door State</TableCell>
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
