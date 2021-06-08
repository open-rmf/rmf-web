/* istanbul ignore file */

import { makeStyles } from '@material-ui/core';
import React from 'react';
import { RobotPanel, VerboseRobot } from 'react-components';
import * as RmfModels from 'rmf-models';
import { FleetStateContext, RmfIngressContext } from '../rmf-app';

const useStyles = makeStyles((theme) => ({
  robotPanel: {
    margin: `${theme.spacing(4)}px auto`,
    width: '100%',
    height: '100%',
    maxWidth: 1600,
  },
}));

export function RobotPage() {
  const classes = useStyles();
  const { fleetsApi } = React.useContext(RmfIngressContext) || {};

  const [verboseRobots, setVerboseRobots] = React.useState<VerboseRobot[]>([]);
  const fetchVerboseRobots = React.useCallback(async () => {
    const resp = await fleetsApi?.getRobotsFleetsRobotsGet(
      undefined,
      undefined,
      undefined,
      undefined,
      'fleet_name,robot_name',
    );
    if (resp) {
      setVerboseRobots(resp.data.items);
    }
  }, [fleetsApi]);

  React.useEffect(() => {
    fetchVerboseRobots();
  }, [fetchVerboseRobots]);

  return (
    <RobotPanel
      className={classes.robotPanel}
      fetchVerboseRobots={fetchVerboseRobots}
      verboseRobots={verboseRobots}
    />
  );
}
