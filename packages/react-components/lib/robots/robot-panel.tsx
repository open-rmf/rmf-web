import React from 'react';
import { Grid, makeStyles, Paper, Typography, TablePagination } from '@material-ui/core';
import { RobotInfo } from './robot-info';
import { RobotTable } from './robot-table';
import { VerboseRobot } from './utils';

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
  paginationOptions?: Omit<React.ComponentPropsWithoutRef<typeof TablePagination>, 'component'>;
  verboseRobots: VerboseRobot[];
  fetchVerboseRobots: () => Promise<VerboseRobot[]>;
}

export function RobotPanel({
  paginationOptions,
  verboseRobots,
  fetchVerboseRobots,
  ...divProps
}: RobotPanelProps): JSX.Element {
  const classes = useStyles();
  const [selectedRobot, setSelectedRobot] = React.useState<VerboseRobot | undefined>(undefined);

  const handleRefresh = async (selectedRobot?: VerboseRobot) => {
    (async () => {
      const result = await fetchVerboseRobots();
      result.forEach((robot) => {
        if (selectedRobot && robot.name === selectedRobot.name) {
          setSelectedRobot(robot);
        }
      });
    })();
  };

  React.useEffect(() => {
    fetchVerboseRobots();
    verboseRobots.forEach((robot) => {
      if (selectedRobot && robot.fleet + robot.name === selectedRobot.fleet + selectedRobot.name)
        setSelectedRobot(robot);
    });
  }, [verboseRobots, fetchVerboseRobots]);

  return (
    <div {...divProps}>
      <Grid container wrap="nowrap" justify="center" style={{ height: 'inherit' }}>
        <Grid style={{ flex: '1 1 auto' }}>
          <RobotTable
            className={classes.robotTable}
            robots={verboseRobots}
            paginationOptions={paginationOptions}
            onRobotClick={(_ev, robot) => setSelectedRobot(robot)}
            onRefreshClick={() => handleRefresh(selectedRobot)}
          />
        </Grid>
        <Paper className={classes.detailPanelContainer}>
          {selectedRobot ? <RobotInfo robot={selectedRobot} /> : <NoSelectedRobot />}
        </Paper>
      </Grid>
    </div>
  );
}
