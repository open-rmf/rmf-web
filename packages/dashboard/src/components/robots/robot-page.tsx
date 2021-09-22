/* istanbul ignore file */

import { styled } from '@material-ui/core';
import React from 'react';
import { RobotPanel, RobotPanelProps, VerboseRobot } from 'react-components';
import { RmfIngressContext } from '../rmf-app';

const classes = {
  robotPanel: 'robot-page-container',
};

const RobotPageRoot = styled((props: RobotPanelProps) => <RobotPanel {...props} />)(
  ({ theme }) => ({
    [`&.${classes.robotPanel}`]: {
      margin: `${theme.spacing(4)}px auto`,
      width: '100%',
      height: '100%',
      maxWidth: 1600,
    },
  }),
);

export function RobotPage() {
  const { fleetsApi } = React.useContext(RmfIngressContext) || {};

  const [hasMore, setHasMore] = React.useState(true);
  const [page, setPage] = React.useState(0);
  const [verboseRobots, setVerboseRobots] = React.useState<VerboseRobot[]>([]);
  const fetchVerboseRobots = React.useCallback(async () => {
    if (!fleetsApi) {
      setHasMore(false);
      return [];
    }
    const resp = await fleetsApi?.getRobotsFleetsRobotsGet(
      undefined,
      undefined,
      11,
      page * 10,
      'fleet_name,robot_name',
    );
    const robots = resp.data as VerboseRobot[];
    setHasMore(robots.length > 10);
    const slicedRobots = robots.slice(0, 10);
    setVerboseRobots(slicedRobots);
    return slicedRobots;
  }, [fleetsApi, page]);

  React.useEffect(() => {
    fetchVerboseRobots();
  }, [fetchVerboseRobots]);

  return (
    <RobotPageRoot
      className={classes.robotPanel}
      fetchVerboseRobots={fetchVerboseRobots}
      paginationOptions={{
        count: hasMore ? -1 : page * 10 + verboseRobots.length,
        rowsPerPage: 10,
        rowsPerPageOptions: [10],
        page,
        onPageChange: (_ev, newPage) => setPage(newPage),
      }}
      verboseRobots={verboseRobots}
    />
  );
}
