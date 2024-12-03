import { Dialog, DialogContent, DialogTitle, Divider, TextField, useTheme } from '@mui/material';
import React from 'react';
import { Door as DoorModel } from 'rmf-models/ros/rmf_building_map_msgs/msg';

import { useRmfApi } from '../../hooks';
import { getApiErrorMessage } from '../../utils/api';
import { DoorTableData } from './door-table-datagrid';
import { doorModeToString, doorTypeToString } from './door-utils';

interface DoorSummaryProps {
  onClose: () => void;
  door: DoorModel;
  doorLevelName: string;
}

export const DoorSummary = ({ onClose, door, doorLevelName }: DoorSummaryProps): JSX.Element => {
  const rmfApi = useRmfApi();
  const [doorData, setDoorData] = React.useState<DoorTableData>({
    index: 0,
    doorName: door.name,
    levelName: doorLevelName,
    doorType: door.door_type,
    doorState: undefined,
  });

  React.useEffect(() => {
    const fetchDataForDoor = async () => {
      try {
        const sub = rmfApi.getDoorStateObs(door.name).subscribe((doorState) => {
          setDoorData({
            index: 0,
            doorName: door.name,
            levelName: doorLevelName,
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
  }, [rmfApi, doorLevelName, door]);

  const [isOpen, setIsOpen] = React.useState(true);

  const theme = useTheme();

  return (
    <Dialog
      PaperProps={{
        style: {
          boxShadow: 'none',
          background: theme.palette.info.main,
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
            return <div key={doorData.doorName + key} />;
          }
          let displayValue = value;
          let displayLabel = key;
          switch (key) {
            case 'doorName':
              displayLabel = 'Name';
              break;
            case 'levelName':
              displayLabel = 'Current Floor';
              break;
            case 'doorType':
              displayValue = doorTypeToString(value);
              displayLabel = 'Type';
              break;
            case 'doorState':
              displayValue = value
                ? doorModeToString(value.current_mode.value)
                : doorModeToString(undefined);
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
