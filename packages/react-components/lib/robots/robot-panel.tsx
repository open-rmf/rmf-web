import { Grid, Paper, TablePagination, Typography, styled } from '@mui/material';
import { RobotState, TaskState } from 'api-client';
import React from 'react';
import { RobotInfo } from './robot-info';
import { RobotTable } from './robot-table';
import { VerboseRobot } from './utils';

const classes = {
  detailPanelContainer: 'robot-panel-detail-container',
  robotTable: 'robot-panel-table',
};
const StyledDiv = styled('div')(({ theme }) => ({
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
  verboseRobots: RobotState[];
  fetchVerboseRobots: () => Promise<RobotState[]>;
  fetchSelectedTask?: (taskId: string) => Promise<TaskState | undefined>;
  onRobotZoom?: (robot: VerboseRobot) => void;
}

// FIXME - change fetchVerboseRobots props to onRefresh
// and shift handleRefresh logic to the parent component
export function RobotPanel({
  paginationOptions,
  verboseRobots,
  fetchVerboseRobots,
  fetchSelectedTask,
  onRobotZoom,
  ...divProps
}: RobotPanelProps): JSX.Element {
  const [selectedRobot, setSelectedRobot] = React.useState<RobotState | undefined>(undefined);

  const handleRefresh = async (selectedRobot?: RobotState) => {
    (async () => {
      const result = await fetchVerboseRobots();
      result.forEach((robot) => {
        if (selectedRobot && robot.name === selectedRobot.name) {
          setSelectedRobot(robot);
        }
      });
    })();
  };

  const handleRobotClick = async (
    _ev: React.MouseEvent<HTMLDivElement, MouseEvent>,
    robot: VerboseRobot,
  ) => {
    await handleRefresh(robot);
    setSelectedRobot(robot);
    onRobotZoom && onRobotZoom(robot);
  };

  return (
    <StyledDiv {...divProps}>
      <Grid container wrap="nowrap" justifyContent="center" style={{ height: 'inherit' }}>
        <Grid style={{ flex: '1 1 auto' }}>
          <RobotTable
            className={classes.robotTable}
            robots={verboseRobots}
            paginationOptions={paginationOptions}
            onRobotClick={handleRobotClick}
            onRefreshClick={() => handleRefresh(selectedRobot)}
          />
        </Grid>
        <Paper className={classes.detailPanelContainer}>
          {selectedRobot ? (
            <RobotInfo robot={selectedRobot} fetchSelectedTask={fetchSelectedTask} />
          ) : (
            <NoSelectedRobot />
          )}
        </Paper>
      </Grid>
    </StyledDiv>
  );
}
