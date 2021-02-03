import React from 'react';
import { makeStyles } from '@material-ui/core';
import Button from '@material-ui/core/Button';

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
        Status: {emergencyState}
      </Button>
    </div>
  );
};

export default EmergencyPanel;
