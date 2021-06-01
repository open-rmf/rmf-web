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
import { TaskProgress } from 'api-client';

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
  const taskDescription = robot.assignedTasks[0].task_summary.task_profile.description;
  switch (taskTypeToStr(taskDescription.task_type.type)) {
    case 'Loop':
      return (
        <>
          <TableCell>{taskDescription.loop.start_name}</TableCell>
          <TableCell>{taskDescription.loop.finish_name}</TableCell>
        </>
      );
    case 'Delivery':
      return (
        <>
          <TableCell>{taskDescription.delivery.pickup_place_name}</TableCell>
          <TableCell>{taskDescription.delivery.dropoff_place_name}</TableCell>
        </>
      );
    case 'Clean':
      return (
        <>
          <TableCell>-</TableCell>
          <TableCell>{taskDescription.clean.start_waypoint}</TableCell>
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

  if (robot.assignedTasks.length == 0) {
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
            {robot.assignedTasks
              ? robot.assignedTasks[0].task_summary.end_time.sec -
                robot.assignedTasks[0].task_summary.start_time.sec
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
  tasks: TaskProgress[];
  robots: RmfModels.RobotState[];
  paginationOptions?: PaginationOptions;
  onRefreshTasks?: (limt?: number, offset?: number, robotName?: string) => void;
  onRobotClickAndRefresh?(robot: VerboseRobot, ev?: React.MouseEvent<HTMLDivElement>): void;
}

export function RobotTable({
  tasks,
  robots,
  paginationOptions,
  onRefreshTasks,
  onRobotClickAndRefresh,
  ...paperProps
}: RobotTableProps): JSX.Element {
  const classes = useStyles();
  const [robotsWithTasks, setRobotsWithTasks] = React.useState<VerboseRobot[]>([]);
  const [selectedRobot, setSelectedRobot] = React.useState<VerboseRobot | undefined>(undefined);

  React.useEffect(() => {
    setRobotsWithTasks(allocateTasksToRobots(robots, tasks));
  }, [robots, tasks]);

  const useInterval = (callback: () => void, delay: number) => {
    const savedCallback = React.useRef<() => void>();

    React.useEffect(() => {
      savedCallback.current = callback;
    }, [callback]);

    React.useEffect(() => {
      const tick = () => {
        savedCallback.current && savedCallback.current();
      };
      const id = setInterval(tick, delay);
      return () => clearInterval(id);
    }, [delay]);
  };

  const updateRobotWithTask = () => {
    onRefreshTasks && onRefreshTasks(undefined, undefined, selectedRobot?.name);
    setRobotsWithTasks(allocateTasksToRobots(robots, tasks));
    if (robotsWithTasks.length > 0) {
      robotsWithTasks.forEach((robot) => {
        if (robot.name === selectedRobot?.name)
          onRobotClickAndRefresh && onRobotClickAndRefresh(robot);
      });
    }
  };

  useInterval(updateRobotWithTask, 3000);

  return (
    <Paper {...paperProps}>
      <Toolbar>
        <Typography className={classes.title} variant="h6">
          Robots
        </Typography>
        <IconButton
          onClick={() => {
            onRefreshTasks;
          }}
          aria-label="Refresh"
        >
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
                  onClick={(ev) => {
                    setSelectedRobot(robot);
                    onRefreshTasks && onRefreshTasks(undefined, undefined, robot.name);
                    onRobotClickAndRefresh && onRobotClickAndRefresh(robot, ev);
                  }}
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
