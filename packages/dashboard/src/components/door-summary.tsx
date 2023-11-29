import React from 'react';
import { Dialog, DialogContent, DialogTitle, Divider, TextField } from '@mui/material';
import { Theme } from '@mui/material/styles';
import { Door as DoorModel } from 'rmf-models';
import { RmfAppContext } from './rmf-app';
import { getApiErrorMessage } from './utils';
import {
  doorModeToString,
  DoorTableData,
  doorTypeToString,
  healthStatusToOpMode,
  base,
} from 'react-components';
import { Level } from 'api-client';
import { makeStyles, createStyles } from '@mui/styles';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    textField: {
      background: theme.palette.background.default,
      '&:hover': {
        backgroundColor: theme.palette.background.default,
      },
    },
  }),
);

interface DoorSummaryProps {
  onClose: () => void;
  door: DoorModel;
  level: Level;
}

export const DoorSummary = ({ onClose, door, level }: DoorSummaryProps): JSX.Element => {
  const classes = useStyles();
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
              displayValue = healthStatusToOpMode(value);
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
                InputProps={{ readOnly: true, className: classes.textField }}
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
