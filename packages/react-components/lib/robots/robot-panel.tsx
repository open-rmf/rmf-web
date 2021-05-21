import React from 'react';
import { Grid, makeStyles, Paper, Typography } from '@material-ui/core';
import * as RmfModels from 'rmf-models';
import { RobotInfo } from './robot-info';
import { RobotTable, RobotTableProps } from './robot-table';
import { VerboseRobot } from './utils';
import { Task } from 'api-client';

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
  tasks: Task[];
  robots: RmfModels.RobotState[];
  onRefreshClick?: RobotTableProps['onRefreshClick'];
}

export function RobotPanel({
  tasks,
  robots,
  onRefreshClick,
  ...divProps
}: RobotPanelProps): JSX.Element {
  const classes = useStyles();
  const [page, setPage] = React.useState(0);
  const [selectedRobot, setSelectedRobot] = React.useState<VerboseRobot | undefined>(undefined);

  return (
    <div {...divProps}>
      <Grid container wrap="nowrap" justify="center" style={{ height: 'inherit' }}>
        <Grid style={{ flex: '1 1 auto' }}>
          <RobotTable
            className={classes.robotTable}
            tasks={tasks}
            robots={robots.slice(page * 10, (page + 1) * 10)}
            paginationOptions={{
              count: tasks.length,
              rowsPerPage: 10,
              rowsPerPageOptions: [10],
              page,
              onChangePage: (_ev, newPage) => setPage(newPage),
            }}
            onRobotClick={(_ev, robot) => setSelectedRobot(robot)}
            onRefreshClick={onRefreshClick}
          />
        </Grid>
        <Paper className={classes.detailPanelContainer}>
          {selectedRobot ? <RobotInfo robot={selectedRobot} /> : <NoSelectedRobot />}
        </Paper>
      </Grid>
    </div>
  );
}
