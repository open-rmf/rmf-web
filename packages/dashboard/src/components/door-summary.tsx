import React from 'react';
import { Dialog, DialogContent, DialogTitle, Divider, Typography } from '@mui/material';
import { useTheme } from '@mui/material/styles';
import { Door as DoorModel } from 'rmf-models';
import { RmfAppContext } from './rmf-app';
import { getApiErrorMessage } from './utils';
import {
  doorModeToString,
  DoorTableData,
  doorTypeToString,
  healthStatusToOpMode,
} from 'react-components';
import { Level } from 'api-client';

interface DoorSummaryProps {
  onClose: () => void;
  door: DoorModel;
  level: Level;
}

export const DoorSummary = ({ onClose, door, level }: DoorSummaryProps): JSX.Element => {
  const theme = useTheme();
  const rmf = React.useContext(RmfAppContext);
  const [doorData, setDoorData] = React.useState<DoorTableData>({
    index: 0,
    doorName: '',
    opMode: '',
    levelName: '',
    doorType: 0,
    doorState: undefined,
  });

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const fetchDataForDoor = async () => {
      try {
        const { data } = await rmf.doorsApi.getDoorHealthDoorsDoorNameHealthGet(door.name);
        const { health_status } = data;
        const sub = rmf.getDoorStateObs(door.name).subscribe((doorState) => {
          setDoorData({
            index: 0,
            doorName: door.name,
            opMode: health_status ? health_status : 'N/A',
            levelName: level.name,
            doorType: door.door_type,
            doorState: doorState,
          });
        });
        return () => sub.unsubscribe();
      } catch (error) {
        console.error(`Failed to get door health: ${getApiErrorMessage(error)}`);
      }
    };

    fetchDataForDoor();
  }, [rmf, level, door]);

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
      <DialogTitle align="center">Door summary</DialogTitle>
      <Divider />
      <DialogContent>
        <Typography variant="body1">
          <strong>Name:</strong> {doorData.doorName}
        </Typography>
        <Typography variant="body1">
          <strong>Op. Mode:</strong> {healthStatusToOpMode(doorData.opMode)}
        </Typography>
        <Typography variant="body1">
          <strong>Current Floor:</strong> {doorData.levelName}
        </Typography>
        <Typography variant="body1">
          <strong>Type:</strong> {doorTypeToString(door.door_type)}
        </Typography>
        <Typography variant="body1">
          <strong>State:</strong>{' '}
          {doorData.doorState ? doorModeToString(doorData.doorState.current_mode.value) : -1}
        </Typography>
      </DialogContent>
    </Dialog>
  );
};
