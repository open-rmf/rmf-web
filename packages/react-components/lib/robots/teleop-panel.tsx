import { Grid, makeStyles, Paper, TablePagination, Typography } from '@material-ui/core';
import React from 'react';
import { TeleopInfo } from './teleop-info';
import { TeleopTable } from './teleop-table';
import { VerboseRobot } from './utils';

const useStyles = makeStyles((theme) => ({
  detailPanelContainer: {
    width: 900,
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

export interface TeleopPanelProps extends React.HTMLProps<HTMLDivElement> {
  paginationOptions?: Omit<React.ComponentPropsWithoutRef<typeof TablePagination>, 'component'>;
  verboseRobots: VerboseRobot[];
  fetchVerboseRobots: () => Promise<VerboseRobot[]>;
}

export function TeleopPanel({
  paginationOptions,
  verboseRobots,
  fetchVerboseRobots,
  ...divProps
}: TeleopPanelProps): JSX.Element {
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

  return (
    <div {...divProps}>
      <Grid container wrap="nowrap" justify="center" style={{ height: 'inherit' }}>
        <Grid style={{ flex: '1 1 auto' }}>
          <TeleopTable
            className={classes.robotTable}
            robots={verboseRobots}
            paginationOptions={paginationOptions}
            onRobotClick={(_ev, robot) => setSelectedRobot(robot)}
            onRefreshClick={() => handleRefresh(selectedRobot)}
          />
        </Grid>
        <Paper className={classes.detailPanelContainer}>
          {selectedRobot ? <TeleopInfo robot={selectedRobot} /> : <NoSelectedRobot />}
        </Paper>
      </Grid>
    </div>
  );
}
