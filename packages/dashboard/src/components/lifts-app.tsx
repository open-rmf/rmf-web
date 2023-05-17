import {
  SxProps,
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  Typography,
  useTheme,
} from '@mui/material';
import { BuildingMap, Lift, LiftState } from 'api-client';
import ArrowDownwardIcon from '@mui/icons-material/ArrowDownward';
import ArrowUpwardIcon from '@mui/icons-material/ArrowUpward';
import React from 'react';
import { LiftControls, doorStateToString, motionStateToString } from 'react-components';
import { createMicroApp } from './micro-app';
import { RmfAppContext } from './rmf-app';
import { LiftState as LiftStateModel } from 'rmf-models';

interface LiftTableRowProps {
  lift: Lift;
}

const LiftTableRow = ({ lift }: LiftTableRowProps) => {
  const rmf = React.useContext(RmfAppContext);
  const [liftState, setLiftState] = React.useState<LiftState | null>(null);
  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.getLiftStateObs(lift.name).subscribe(setLiftState);
    return () => sub.unsubscribe();
  }, [rmf, lift.name]);

  const theme = useTheme();
  const currMotion = motionStateToString(liftState?.motion_state);
  const motionArrowActiveStyle: SxProps = {
    color: theme.palette.primary.main,
  };
  const motionArrowIdleStyle: SxProps = {
    color: theme.palette.action.disabled,
    opacity: theme.palette.action.disabledOpacity,
  };
  const currDoorMotion = doorStateToString(liftState?.door_state);

  const doorStateLabelStyle: SxProps = (() => {
    switch (liftState?.door_state) {
      case LiftStateModel.DOOR_OPEN:
        return {
          color: theme.palette.success.main,
        };
      case LiftStateModel.DOOR_CLOSED:
        return {
          color: theme.palette.error.main,
        };
      case LiftStateModel.DOOR_MOVING:
        return {
          color: theme.palette.warning.main,
        };
      default:
        return {
          color: theme.palette.action.disabledBackground,
        };
    }
  })();

  return (
    <TableRow key={lift.name}>
      <TableCell>{lift.name}</TableCell>
      <TableCell>{liftState?.current_floor || '?'}</TableCell>
      <TableCell>{liftState?.destination_floor || 'Unknown'}</TableCell>
      <TableCell sx={doorStateLabelStyle}>
        <Typography
          component="p"
          sx={{
            marginRight: liftState?.door_state === LiftStateModel.DOOR_OPEN ? 2 : 0,
            fontWeight: 'bold',
            fontSize: 14,
            display: 'inline-block',
          }}
        >
          {currDoorMotion}
        </Typography>
        <ArrowUpwardIcon sx={currMotion === 'Up' ? motionArrowActiveStyle : motionArrowIdleStyle} />
        <ArrowDownwardIcon
          sx={currMotion === 'Down' ? motionArrowActiveStyle : motionArrowIdleStyle}
        />
      </TableCell>
      <TableCell align="right" sx={{ marginRight: 1, padding: 0, paddingRight: 1 }}>
        <LiftControls
          availableLevels={lift.levels}
          currentLevel={liftState?.current_floor}
          onRequestSubmit={(_ev, doorState, requestType, destination) =>
            rmf?.liftsApi.postLiftRequestLiftsLiftNameRequestPost(lift.name, {
              destination,
              door_mode: doorState,
              request_type: requestType,
            })
          }
        />
      </TableCell>
    </TableRow>
  );
};

export const LiftsApp = createMicroApp('Lifts', () => {
  const rmf = React.useContext(RmfAppContext);
  const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.buildingMapObs.subscribe((newMap) => {
      setBuildingMap(newMap);
    });
    return () => sub.unsubscribe();
  }, [rmf]);

  return (
    <Table stickyHeader size="small" aria-label="door-table" style={{ tableLayout: 'fixed' }}>
      <TableHead>
        <TableRow>
          <TableCell>Name</TableCell>
          <TableCell>Current floor</TableCell>
          <TableCell>Destination floor</TableCell>
          <TableCell>Door state</TableCell>
          <TableCell></TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {buildingMap &&
          buildingMap.lifts.map((lift) => {
            return <LiftTableRow key={lift.name} lift={lift} />;
          })}
      </TableBody>
    </Table>
  );
});
