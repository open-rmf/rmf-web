import { Grid, Paper, TablePagination, Typography, styled } from '@mui/material';
import React from 'react';
import { RobotInfo } from './robot-info';
import { RobotTable } from './robot-table';
import { VerboseRobot } from './utils';

const classes = {
  detailPanelContainer: 'robot-panel-detail-container',
  robotTable: 'robot-panel-table',
};
const StyledRobotPanel = styled('div')(({ theme }) => ({
  [`& .${classes.detailPanelContainer}`]: {
    width: 350,
    padding: theme.spacing(2),
    marginLeft: theme.spacing(2),
    flex: '0 0 auto',
  },
  [`& .${classes.robotTable}`]: {
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
  },
}));

function NoSelectedRobot() {
  return (
    <Grid container wrap="nowrap" alignItems="center" style={{ height: '100%' }}>
      <Typography variant="h6" align="center">
        Click on a robot to view more information
      </Typography>
    </Grid>
  );
}

export interface RobotPanelProps
  extends React.DetailedHTMLProps<React.HTMLAttributes<HTMLDivElement>, HTMLDivElement> {
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

  return (
    <StyledRobotPanel {...divProps}>
      <Grid container wrap="nowrap" justifyContent="center" style={{ height: 'inherit' }}>
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
    </StyledRobotPanel>
  );
}
