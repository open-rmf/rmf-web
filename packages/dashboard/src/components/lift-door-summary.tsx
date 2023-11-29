import React from 'react';
import { Dialog, DialogContent, DialogTitle, Divider, TextField } from '@mui/material';
import { Theme } from '@mui/material/styles';
import { RmfAppContext } from './rmf-app';
import { getApiErrorMessage } from './utils';
import { doorStateToString, healthStatusToOpMode, LiftTableData, base } from 'react-components';
import { Lift } from 'api-client';
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

interface LiftDoorSummaryProps {
  onClose: () => void;
  lift: Lift;
}

export const LiftDoorSummary = ({ onClose, lift }: LiftDoorSummaryProps): JSX.Element => {
  const classes = useStyles();
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
      <DialogTitle align="center">Lift summary</DialogTitle>
      <Divider />
      <DialogContent>
        <DialogContent>
          {Object.entries(liftDoorData).map(([key, value]) => {
            if (key === 'index' || key === 'motionState' || key === 'lift') {
              return <></>;
            }
            let displayValue = value;
            let displayLabel = key;
            switch (key) {
              case 'name':
                displayLabel = 'Name';
                break;
              case 'opMode':
                displayValue = healthStatusToOpMode(value);
                displayLabel = 'Op. Mode';
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
              default:
                break;
            }
            return (
              <div key={liftDoorData.name + key}>
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
      </DialogContent>
    </Dialog>
  );
};
