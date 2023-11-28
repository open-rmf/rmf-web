import React from 'react';
import { Dialog, DialogContent, DialogTitle, Divider, Typography } from '@mui/material';
import { useTheme } from '@mui/material/styles';
import { RmfAppContext } from './rmf-app';
import { getApiErrorMessage } from './utils';
import { doorStateToString, healthStatusToOpMode, LiftTableData } from 'react-components';
import { Lift } from 'api-client';

interface LiftDoorSummaryProps {
  onClose: () => void;
  lift: Lift;
}

export const LiftDoorSummary = ({ onClose, lift }: LiftDoorSummaryProps): JSX.Element => {
  const theme = useTheme();
  const rmf = React.useContext(RmfAppContext);
  const [liftDoorData, setLiftDoorData] = React.useState<LiftTableData>({
    index: 0,
    name: '',
    opMode: '',
    currentFloor: '',
    destinationFloor: '',
    doorState: 0,
    motionState: 0,
    lift: lift,
  });

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const fetchDataForLift = async () => {
      try {
        const { data } = await rmf.liftsApi.getLiftHealthLiftsLiftNameHealthGet(lift.name);
        const { health_status } = data;
        const sub = rmf.getLiftStateObs(lift.name).subscribe((liftState) => {
          setLiftDoorData({
            index: -1,
            name: lift.name,
            opMode: health_status ? health_status : 'N/A',
            currentFloor: liftState.current_floor,
            destinationFloor: liftState.destination_floor,
            doorState: liftState.door_state,
            motionState: liftState.motion_state,
            lift: lift,
          });
        });
        return () => sub.unsubscribe();
      } catch (error) {
        console.error(`Failed to get door health: ${getApiErrorMessage(error)}`);
      }
    };

    fetchDataForLift();
  }, [rmf, lift]);

  const [isOpen, setIsOpen] = React.useState(true);
  return (
    <Dialog
      PaperProps={{
        style: {
          boxShadow: 'none',
          background: theme.palette.background.paper,
        },
      }}
      open={isOpen}
      onClose={() => {
        setIsOpen(false);
        onClose();
      }}
      fullWidth
      maxWidth="sm"
    >
      <DialogTitle align="center">Lift summary</DialogTitle>
      <Divider />
      <DialogContent>
        <Typography variant="body1">Name: {liftDoorData.name}</Typography>
        <Typography variant="body1">
          Op. Mode: {healthStatusToOpMode(liftDoorData.opMode)}
        </Typography>
        <Typography variant="body1">Current Floor: {liftDoorData.currentFloor}</Typography>
        <Typography variant="body1">Destination Floor: {liftDoorData.destinationFloor}</Typography>
        <Typography variant="body1">State: {doorStateToString(liftDoorData.doorState)}</Typography>
      </DialogContent>
    </Dialog>
  );
};
