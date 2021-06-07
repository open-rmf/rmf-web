import React from 'react';
import { Grid, makeStyles, Paper, Typography } from '@material-ui/core';
import * as RmfModels from 'rmf-models';
import { RobotInfo } from './robot-info';
import { RobotTable } from './robot-table';
import { VerboseRobot } from './utils';
import { TaskProgress } from 'api-client';

const useStyles = makeStyles((theme) => ({
  detailPanelContainer: {
    width: 350,
    padding: theme.spacing(2),
    marginLeft: theme.spacing(1),
    flex: '0 0 auto',
  },
  robotTable: {
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
  },
}));

function NoSelectedRobot() {
  return (
    <Grid container wrap="nowrap" alignItems="center" style={{ height: '100%' }}>
      <Typography variant="h6" align="center" color="textSecondary">
        Click on a robot to view more information
      </Typography>
    </Grid>
  );
}

export interface RobotPanelProps extends React.HTMLProps<HTMLDivElement> {
  robots: RmfModels.RobotState[];
  verboseRobots?: VerboseRobot[];
  fetchTasks: (limit: number, offset: number) => Promise<TaskProgress[]>;
}

export function RobotPanel({ robots, fetchTasks, ...divProps }: RobotPanelProps): JSX.Element {
  const classes = useStyles();
  const [tasks, setTasks] = React.useState<TaskProgress[]>([]);
  const [totalCount, setTotalCount] = React.useState(-1);
  const [page, setPage] = React.useState(0);
  const [selectedRobot, setSelectedRobot] = React.useState<VerboseRobot | undefined>(undefined);

  const handleRefresh = React.useCallback(async () => {
    (async () => {
      const result = await fetchTasks(10, page * 10);
      setTasks(result);
    })();
  }, [fetchTasks, page]);

  React.useEffect(() => {
    setTotalCount(robots.length);
    handleRefresh();
  }, [handleRefresh, robots]);

  return (
    <div {...divProps}>
      <Grid container wrap="nowrap" justify="center" style={{ height: 'inherit' }}>
        <Grid style={{ flex: '1 1 auto' }}>
          <RobotTable
            className={classes.robotTable}
            tasks={tasks}
            robots={robots.slice(page * 10, (page + 1) * 10)}
            paginationOptions={{
              count: totalCount,
              rowsPerPage: 10,
              rowsPerPageOptions: [10],
              page,
              onChangePage: (_ev, newPage) => setPage(newPage),
            }}
            onRobotClick={(_ev, robot) => setSelectedRobot(robot)}
            onRefreshClick={handleRefresh}
          />
        </Grid>
        <Paper className={classes.detailPanelContainer}>
          {selectedRobot ? <RobotInfo robot={selectedRobot} /> : <NoSelectedRobot />}
        </Paper>
      </Grid>
    </div>
  );
}
