import {
  IconButton,
  makeStyles,
  Paper,
  PaperProps,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TablePagination,
  TableRow,
  Toolbar,
  Typography,
} from '@material-ui/core';
import { Refresh as RefreshIcon } from '@material-ui/icons';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { taskTypeToStr } from '../tasks/utils';
import { robotModeToString, allocateTasksToRobots, VerboseRobot } from './utils';
import { PaginationOptions } from '../tasks/task-table';

const useStyles = makeStyles((theme) => ({
  table: {
    minWidth: 650,
  },
  title: {
    flex: '1 1 100%',
  },
  taskRowHover: {
    background: theme.palette.action.hover,
  },
  infoRow: {
    '& > *': {
      borderBottom: 'unset',
    },
  },
  phasesCell: {
    padding: `0 ${theme.spacing(1)}px`,
  },
}));

interface RobotRowProps {
  robot: VerboseRobot;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

const returnLocationCells = (robot: VerboseRobot) => {
  switch (taskTypeToStr(robot.assigned_tasks[0].task_profile.description.task_type.type)) {
    case 'Loop':
      return (
        <>
          <TableCell>{robot.assigned_tasks[0].task_profile.description.loop.start_name}</TableCell>
          <TableCell>{robot.assigned_tasks[0].task_profile.description.loop.finish_name}</TableCell>
        </>
      );
    case 'Delivery':
      return (
        <>
          <TableCell>
            {robot.assigned_tasks[0].task_profile.description.delivery.pickup_place_name}
          </TableCell>
          <TableCell>
            {robot.assigned_tasks[0].task_profile.description.delivery.dropoff_place_name}
          </TableCell>
        </>
      );
    case 'Clean':
      return (
        <>
          <TableCell>-</TableCell>
          <TableCell>
            {robot.assigned_tasks[0].task_profile.description.clean.start_waypoint}
          </TableCell>
        </>
      );
    default:
      return (
        <>
          <TableCell>-</TableCell>
          <TableCell>-</TableCell>
        </>
      );
  }
};

function RobotRow({ robot, onClick }: RobotRowProps) {
  const classes = useStyles();

  if (robot.assigned_tasks.length == 0) {
    return (
      <>
        <TableRow className={classes.infoRow} onClick={onClick}>
          <TableCell>{robot.name}</TableCell>
          <TableCell>{'-'}</TableCell>
          <TableCell>{'-'}</TableCell>
          <TableCell>{'-'}</TableCell>
          <TableCell>{robot.battery_percent.toFixed(2)}%</TableCell>
          <TableCell>{robotModeToString(robot.mode)}</TableCell>
        </TableRow>
      </>
    );
  } else {
    return (
      <>
        <TableRow className={classes.infoRow} onClick={onClick}>
          <TableCell>{robot.name}</TableCell>
          {returnLocationCells(robot)}
          <TableCell>
            {robot.assigned_tasks
              ? robot.assigned_tasks[0].end_time.sec - robot.assigned_tasks[0].start_time.sec
              : '-'}
          </TableCell>
          <TableCell>{robot.battery_percent.toFixed(2)}%</TableCell>
          <TableCell>{robotModeToString(robot.mode)}</TableCell>
        </TableRow>
      </>
    );
  }
}

export interface RobotTableProps extends PaperProps {
  /**
   * The current list of robots to display, when pagination is enabled, this should only
   * contain the robots for the current page.
   */
  tasks: RmfModels.TaskSummary[];
  robots: RmfModels.RobotState[];
  paginationOptions?: PaginationOptions;
  onRefreshClick?: React.MouseEventHandler<HTMLButtonElement>;
  onRobotClick?(ev: React.MouseEvent<HTMLDivElement>, robot: VerboseRobot): void;
}

export function RobotTable({
  tasks,
  robots,
  paginationOptions,
  onRefreshClick,
  onRobotClick,
  ...paperProps
}: RobotTableProps): JSX.Element {
  const classes = useStyles();
  const [robotsWithTasks, setRobotsWithTasks] = React.useState(
    allocateTasksToRobots(robots, tasks),
  );

  React.useEffect(() => {
    setRobotsWithTasks(allocateTasksToRobots(robots, tasks));
  }, [robots, tasks]);

  return (
    <Paper {...paperProps}>
      <Toolbar>
        <Typography className={classes.title} variant="h6">
          Robots
        </Typography>
        <IconButton onClick={onRefreshClick} aria-label="Refresh">
          <RefreshIcon />
        </IconButton>
      </Toolbar>
      <TableContainer style={{ flex: '1 1 auto' }}>
        <Table className={classes.table} stickyHeader size="small" style={{ tableLayout: 'fixed' }}>
          <TableHead>
            <TableRow>
              <TableCell>Robot Name</TableCell>
              <TableCell>Start Location</TableCell>
              <TableCell>Destination</TableCell>
              <TableCell>End Time</TableCell>
              <TableCell>Battery</TableCell>
              <TableCell>State</TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            {robotsWithTasks &&
              robotsWithTasks.map((robot, robot_id) => (
                <RobotRow
                  key={robot_id}
                  robot={robot}
                  onClick={(ev) => onRobotClick && onRobotClick(ev, robot)}
                />
              ))}
          </TableBody>
        </Table>
      </TableContainer>
      {paginationOptions && (
        <TablePagination component="div" {...paginationOptions} style={{ flex: '0 0 auto' }} />
      )}
    </Paper>
  );
}
