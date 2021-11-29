/* istanbul ignore file */

import { styled } from '@mui/material';
import React from 'react';
import { RobotPanel, RobotPanelProps, VerboseRobot } from 'react-components';
import { RmfIngressContext } from '../rmf-app';

const classes = {
  robotPanel: 'robot-page-container',
};
const StyledRobotPanel = styled((props: RobotPanelProps) => <RobotPanel {...props} />)(
  ({ theme }) => ({
    [`&.${classes.robotPanel}`]: {
      padding: `${theme.spacing(4)}`,
      height: '100%',
      maxWidth: 1600,
      backgroundColor: theme.palette.background.default,
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
    <StyledRobotPanel
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
