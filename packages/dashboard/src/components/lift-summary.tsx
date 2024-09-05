import {
  Dialog,
  DialogContent,
  DialogTitle,
  Divider,
  TextField,
  useMediaQuery,
  useTheme,
} from '@mui/material';
import { Lift } from 'api-client';
import React from 'react';
import { base, doorStateToString, liftModeToString, LiftTableData } from 'react-components';

import { useRmfApi } from '../hooks/use-rmf-api';
import { getApiErrorMessage } from '../utils/api';

interface LiftSummaryProps {
  onClose: () => void;
  lift: Lift;
}

export const LiftSummary = ({ onClose, lift }: LiftSummaryProps): JSX.Element => {
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  const rmfApi = useRmfApi();
  const [liftData, setLiftData] = React.useState<LiftTableData>({
    index: 0,
    name: '',
    currentFloor: '',
    destinationFloor: '',
    doorState: 0,
    motionState: 0,
    sessionId: '',
    lift: lift,
    liftState: undefined,
  });

  React.useEffect(() => {
    const fetchDataForLift = async () => {
      try {
        const sub = rmfApi.getLiftStateObs(lift.name).subscribe((liftState) => {
          setLiftData({
            index: -1,
            name: lift.name,
            currentFloor: liftState.current_floor,
            destinationFloor: liftState.destination_floor,
            doorState: liftState.door_state,
            motionState: liftState.motion_state,
            sessionId: liftState.session_id,
            lift: lift,
          });
        });
        return () => sub.unsubscribe();
      } catch (error) {
        console.error(`Failed to get door health: ${getApiErrorMessage(error)}`);
      }
    };

    fetchDataForLift();
  }, [rmfApi, lift]);

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
      maxWidth={isScreenHeightLessThan800 ? 'xs' : 'sm'}
    >
      <DialogTitle
        align="center"
        sx={{ fontSize: isScreenHeightLessThan800 ? '1.2rem' : '1.5rem' }}
      >
        Lift summary
      </DialogTitle>
      <Divider />
      <DialogContent>
        {Object.entries(liftData).map(([key, value]) => {
          if (key === 'index' || key === 'motionState' || key === 'lift') {
            return <></>;
          }
          let displayValue = value;
          let displayLabel = key;
          switch (key) {
            case 'name':
              displayLabel = 'Name';
              break;
            case 'mode':
              displayValue = liftModeToString(value);
              displayLabel = 'Mode';
              break;
            case 'currentFloor':
              displayLabel = 'Current Floor';
              break;
            case 'destinationFloor':
              displayLabel = 'Destination Floor';
              break;
            case 'doorState':
              displayValue = doorStateToString(value);
              displayLabel = 'State';
              break;
            case 'sessionId':
              displayLabel = 'Session ID';
              break;
            default:
              break;
          }
          return (
            <div key={liftData.name + key}>
              <TextField
                label={displayLabel}
                id="standard-size-small"
                size="small"
                variant="filled"
                InputProps={{ readOnly: true }}
                fullWidth={true}
                multiline
                maxRows={4}
                margin="dense"
                value={displayValue}
                sx={{
                  '& .MuiFilledInput-root': {
                    fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
                  },
                  background: theme.palette.background.default,
                  '&:hover': {
                    backgroundColor: theme.palette.background.default,
                  },
                }}
              />
            </div>
          );
        })}
      </DialogContent>
    </Dialog>
  );
};
