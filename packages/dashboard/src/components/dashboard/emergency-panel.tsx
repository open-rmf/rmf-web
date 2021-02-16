import React from 'react';
import { makeStyles } from '@material-ui/core';
import Button from '@material-ui/core/Button';
import Card from '@material-ui/core/Card';
import CardContent from '@material-ui/core/CardContent';
import Divider from '@material-ui/core/Divider';
import Typography from '@material-ui/core/Typography';
import { Alert, AlertTitle } from '@material-ui/lab';
import StyledChip from '../styled-chip';

/*
The Emergency Panel subscribes to the node that publishes an alarm state.
With the alarm state, active robots can be triggered to move towards
an emergency / nearest docking station so that they are kept out of the way.
*/

const useStyles = makeStyles((theme) => ({
  mainContainer: {
    width: '100%',
    height: '100%',
    display: 'flex',
    flexFlow: 'column',
    borderRadius: 'inherit',
  },
  viewContainer: {
    width: '100%',
    height: '100%',
    position: 'relative',
    overflow: 'hidden',
  },
  statusButton: {
    color: theme.palette.text.primary,
  },
  messageContainer: {
    width: '100%',
    display: 'flex',
    marginTop: '1em',
    marginBottom: '1em',
  },
  buttonContainer: {
    width: '100%',
    display: 'flex',
  },
  triggerButton: {
    '@media (min-aspect-ratio: 8/10)': {
      width: '65%',
      textAlign: 'center',
      margin: 'auto',
    },
    '@media (max-aspect-ratio: 8/10)': {
      width: '35%',
      textAlign: 'center',
      margin: 'auto',
    },
  },
  cardRoot: {
    width: '90%',
    margin: 'auto',
  },
  pos: {
    marginBottom: 12,
  },
}));

export interface EmergencyPanelProps extends React.HTMLProps<HTMLDivElement> {
  emergencyState: string;
}
export const EmergencyPanel = (props: EmergencyPanelProps): JSX.Element => {
  const { emergencyState } = props;
  const classes_ = useStyles();

  return (
    <div className={classes_.mainContainer}>
      <Button variant="outlined" className={classes_.statusButton}>
        <StyledChip state={emergencyState} />
      </Button>
      <div className={classes_.messageContainer}>
        <Card className={classes_.cardRoot}>
          <CardContent>
            <Alert severity="error">
              <AlertTitle>Emergency</AlertTitle>
              This is a code-red alert.
            </Alert>
            <Alert severity="error" color="info">
              <AlertTitle>Emergency</AlertTitle>
              This is a code-blue alert.
            </Alert>
            <Divider />
            <Typography variant="body2" component="p">
              Emergency details
            </Typography>
          </CardContent>
        </Card>
      </div>
      <div className={classes_.buttonContainer}>
        <Button
          variant="contained"
          color="primary"
          className={classes_.triggerButton}
          type="submit"
        >
          Dock All Active Robots
        </Button>
      </div>
    </div>
  );
};

export default EmergencyPanel;
