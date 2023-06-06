import { Button, SxProps, Typography, useTheme } from '@mui/material';
import { BuildingMap, DoorState } from 'api-client';
import React from 'react';
import { doorModeToString, doorTypeToString } from 'react-components';
import { DoorMode, DoorMode as RmfDoorMode } from 'rmf-models';
import { createMicroApp } from './micro-app';
import { RmfAppContext } from './rmf-app';
import Table from '@mui/material/Table';
import TableBody from '@mui/material/TableBody';
import TableCell from '@mui/material/TableCell';
import TableHead from '@mui/material/TableHead';
import TableRow from '@mui/material/TableRow';

interface DoorTableRowProps {
  doorName: string;
  levelName: string;
  doorType: number;
}

const DoorTableRow = ({ doorName, levelName, doorType }: DoorTableRowProps) => {
  const theme = useTheme();
  const rmf = React.useContext(RmfAppContext);
  const [doorState, setDoorState] = React.useState<DoorState | null>(null);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.getDoorStateObs(doorName).subscribe(setDoorState);
    return () => sub.unsubscribe();
  }, [rmf, doorName]);

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
    <TableRow key={doorName}>
      <TableCell>{doorName}</TableCell>
      <TableCell>{levelName}</TableCell>
      <TableCell>{doorTypeToString(doorType)}</TableCell>
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
            rmf?.doorsApi.postDoorRequestDoorsDoorNameRequestPost(doorName, {
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
            rmf?.doorsApi.postDoorRequestDoorsDoorNameRequestPost(doorName, {
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
              <DoorTableRow
                key={door.name}
                doorName={door.name}
                levelName={level.name}
                doorType={door.door_type}
              />
            )),
          )}
      </TableBody>
    </Table>
  );
});
