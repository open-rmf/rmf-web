import { SxProps, Table, TableBody, TableCell, TableHead, TableRow, useTheme } from '@mui/material';
import { BuildingMap, LiftState } from 'api-client';
import ArrowDownwardIcon from '@mui/icons-material/ArrowDownward';
import ArrowUpwardIcon from '@mui/icons-material/ArrowUpward';
import React from 'react';
import {
  LiftCardProps as BaseLiftCardProps,
  LiftControls,
  doorStateToString,
  motionStateToString,
} from 'react-components';
import { createMicroApp } from './micro-app';
import { RmfAppContext } from './rmf-app';
import { LiftState as LiftStateModel } from 'rmf-models';

type LiftCardProps = Omit<
  BaseLiftCardProps,
  'motionState' | 'doorState' | 'currentFloor' | 'destinationFloor'
>;

const LiftCard = ({ children, ...otherProps }: LiftCardProps) => {
  const rmf = React.useContext(RmfAppContext);
  const [liftState, setLiftState] = React.useState<LiftState | null>(null);
  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const sub = rmf.getLiftStateObs(otherProps.name).subscribe(setLiftState);
    return () => sub.unsubscribe();
  }, [rmf, otherProps.name]);

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
          backgroundColor: theme.palette.success.main,
          color: theme.palette.success.contrastText,
        };
      case LiftStateModel.DOOR_CLOSED:
        return {
          backgroundColor: theme.palette.error.main,
          color: theme.palette.error.contrastText,
        };
      case LiftStateModel.DOOR_MOVING:
        return {
          backgroundColor: theme.palette.warning.main,
          color: theme.palette.warning.contrastText,
        };
      default:
        return {
          backgroundColor: theme.palette.action.disabledBackground,
          color: theme.palette.action.disabled,
        };
    }
  })();

  return (
    <TableRow key={otherProps.name}>
      <TableCell>{otherProps.name}</TableCell>
      <TableCell>{liftState?.current_floor || '?'}</TableCell>
      <TableCell>{liftState?.destination_floor || 'Unknown'}</TableCell>
      <TableCell sx={doorStateLabelStyle}>{currDoorMotion}</TableCell>
      <TableCell align="center">
        <ArrowUpwardIcon sx={currMotion === 'Up' ? motionArrowActiveStyle : motionArrowIdleStyle} />
        <ArrowDownwardIcon
          sx={currMotion === 'Down' ? motionArrowActiveStyle : motionArrowIdleStyle}
        />
      </TableCell>
      <TableCell align="center" sx={{ margin: 0, padding: 0 }}>
        <LiftControls
          availableLevels={otherProps.lift.levels}
          currentLevel={liftState?.current_floor}
          onRequestSubmit={(_ev, doorState, requestType, destination) =>
            rmf?.liftsApi.postLiftRequestLiftsLiftNameRequestPost(otherProps.lift.name, {
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
          <TableCell></TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {buildingMap &&
          buildingMap.lifts.map((lift) => {
            return <LiftCard key={lift.name} name={lift.name} lift={lift} sx={{ width: 200 }} />;
          })}
      </TableBody>
    </Table>
  );
});
