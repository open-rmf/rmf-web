import { Grid, makeStyles, Paper, TablePagination, Typography } from '@material-ui/core';
import React from 'react';
import { LeafletContext } from 'react-leaflet';
import { RobotInfo } from './robot-info';
import { RobotTable } from './robot-table';
import { VerboseRobot } from './utils';

const useStyles = makeStyles((theme) => ({
  detailPanelContainer: {
    width: 350,
    padding: theme.spacing(2),
    marginLeft: theme.spacing(2),
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
      <Typography variant="h6" align="center">
        Click on a robot to view more information
      </Typography>
    </Grid>
  );
}

export interface RobotPanelProps extends React.HTMLProps<HTMLDivElement> {
  leafletMap?: LeafletContext;
  paginationOptions?: Omit<React.ComponentPropsWithoutRef<typeof TablePagination>, 'component'>;
  verboseRobots: VerboseRobot[];
  fetchVerboseRobots: () => Promise<VerboseRobot[]>;
}

export function RobotPanel({
  paginationOptions,
  verboseRobots,
  leafletMap,
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

  const handleRobotClick = (
    _ev: React.MouseEvent<HTMLDivElement, MouseEvent>,
    robot: VerboseRobot,
  ) => {
    setSelectedRobot(robot);
    leafletMap &&
      leafletMap.map?.setView([robot.state.location.y, robot.state.location.x], 5.5, {
        animate: true,
      });
  };

  return (
    <div {...divProps}>
      <Grid container wrap="nowrap" justify="center" style={{ height: 'inherit' }}>
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
          {selectedRobot ? <RobotInfo robot={selectedRobot} /> : <NoSelectedRobot />}
        </Paper>
      </Grid>
    </div>
  );
}
