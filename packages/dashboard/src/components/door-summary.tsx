import { Dialog, DialogContent, DialogTitle, Divider, TextField, useTheme } from '@mui/material';
import { Level } from 'api-client';
import React from 'react';
import { doorModeToOpModeString } from 'react-components';
import { base, doorModeToString, DoorTableData, doorTypeToString } from 'react-components';
import { Door as DoorModel } from 'rmf-models/ros/rmf_building_map_msgs/msg';

import { useRmfApi } from '../hooks/use-rmf-api';
import { getApiErrorMessage } from '../utils/api';

interface DoorSummaryProps {
  onClose: () => void;
  door: DoorModel;
  level: Level;
}

export const DoorSummary = ({ onClose, door, level }: DoorSummaryProps): JSX.Element => {
  const rmfApi = useRmfApi();
  const [doorData, setDoorData] = React.useState<DoorTableData>({
    index: 0,
    doorName: '',
    levelName: '',
    doorType: 0,
    doorState: undefined,
  });

  React.useEffect(() => {
    const fetchDataForDoor = async () => {
      try {
        const sub = rmfApi.getDoorStateObs(door.name).subscribe((doorState) => {
          setDoorData({
            index: 0,
            doorName: door.name,
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
  }, [rmfApi, level, door]);

  const [isOpen, setIsOpen] = React.useState(true);

  const theme = useTheme();

  return (
    <Dialog
      PaperProps={{
        style: {
          boxShadow: 'none',
          background: base.palette.info.main,
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
        {Object.entries(doorData).map(([key, value]) => {
          if (key === 'index') {
            return <></>;
          }
          let displayValue = value;
          let displayLabel = key;
          switch (key) {
            case 'doorName':
              displayLabel = 'Name';
              break;
            case 'opMode':
              displayValue = doorModeToOpModeString(value.current_mode);
              displayLabel = 'Op. Mode';
              break;
            case 'levelName':
              displayLabel = 'Current Floor';
              break;
            case 'doorType':
              displayValue = doorTypeToString(value);
              displayLabel = 'Type';
              break;
            case 'doorState':
              displayValue = value ? doorModeToString(value.current_mode.value) : -1;
              displayLabel = 'State';
              break;
            default:
              break;
          }
          return (
            <div key={doorData.doorName + key}>
              <TextField
                label={displayLabel}
                id="standard-size-small"
                size="small"
                variant="filled"
                sx={{
                  background: theme.palette.background.default,
                  '&:hover': {
                    backgroundColor: theme.palette.background.default,
                  },
                }}
                InputProps={{ readOnly: true }}
                fullWidth={true}
                multiline
                maxRows={4}
                margin="dense"
                value={displayValue}
              />
            </div>
          );
        })}
      </DialogContent>
    </Dialog>
  );
};
