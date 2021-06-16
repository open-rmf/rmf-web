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
import { TaskProgress } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { taskTypeToStr } from '../tasks/utils';
import { allocateTasksToRobots, robotModeToString, VerboseRobot } from './utils';

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
  tableIcons: {
    color: theme.fontColors,
  },
  tableHeadCell: {
    background: 'rgba(0, 0, 0, 0.1)',
    borderBottom: 'none',
    color: theme.fontColors,
  },
  taskCell: {
    backgroundColor: theme.mainBackground,
    color: theme.fontColors,
  },
}));

interface RobotRowProps {
  robot: VerboseRobot;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

const returnLocationCells = (robot: VerboseRobot, taskTheme: string) => {
  const taskDescription = robot.assignedTasks[0].task_summary.task_profile.description;
  switch (taskTypeToStr(taskDescription.task_type.type)) {
    case 'Loop':
      return (
        <>
          <TableCell className={taskTheme}>{taskDescription.loop.start_name}</TableCell>
          <TableCell className={taskTheme}>{taskDescription.loop.finish_name}</TableCell>
        </>
      );
    case 'Delivery':
      return (
        <>
          <TableCell className={taskTheme}>{taskDescription.delivery.pickup_place_name}</TableCell>
          <TableCell className={taskTheme}>{taskDescription.delivery.dropoff_place_name}</TableCell>
        </>
      );
    case 'Clean':
      return (
        <>
          <TableCell className={taskTheme}>-</TableCell>
          <TableCell className={taskTheme}>{taskDescription.clean.start_waypoint}</TableCell>
        </>
      );
    default:
      return (
        <>
          <TableCell className={taskTheme}>-</TableCell>
          <TableCell className={taskTheme}>-</TableCell>
        </>
      );
  }
};

function RobotRow({ robot, onClick }: RobotRowProps) {
  const classes = useStyles();

  if (robot.assignedTasks.length === 0) {
    return (
      <>
        <TableRow className={classes.taskCell} onClick={onClick}>
          <TableCell className={classes.taskCell}>{robot.name}</TableCell>
          <TableCell className={classes.taskCell}>{'-'}</TableCell>
          <TableCell className={classes.taskCell}>{'-'}</TableCell>
          <TableCell className={classes.taskCell}>{'-'}</TableCell>
          <TableCell className={classes.taskCell}>{robot.battery_percent.toFixed(2)}%</TableCell>
          <TableCell className={classes.taskCell}>{robotModeToString(robot.mode)}</TableCell>
        </TableRow>
      </>
    );
  } else {
    return (
      <>
        <TableRow className={classes.infoRow} onClick={onClick}>
          <TableCell className={classes.taskCell}>{robot.name}</TableCell>
          {returnLocationCells(robot, classes.taskCell)}
          <TableCell className={classes.taskCell}>
            {robot.assignedTasks
              ? `${
                  robot.assignedTasks[0].task_summary.end_time.sec -
                  robot.assignedTasks[0].task_summary.start_time.sec
                }s`
              : '-'}
          </TableCell>
          <TableCell className={classes.taskCell}>{robot.battery_percent.toFixed(2)}%</TableCell>
          <TableCell className={classes.taskCell}>{robotModeToString(robot.mode)}</TableCell>
        </TableRow>
      </>
    );
  }
}

export type PaginationOptions = Omit<
  React.ComponentPropsWithoutRef<typeof TablePagination>,
  'component'
>;

export interface RobotTableProps extends PaperProps {
  /**
   * The current list of robots to display, when pagination is enabled, this should only
   * contain the robots for the current page.
   */
  tasks: TaskProgress[];
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
  const [robotsWithTasks, setRobotsWithTasks] = React.useState<VerboseRobot[]>([]);

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
          <RefreshIcon className={classes.tableIcons} />
        </IconButton>
      </Toolbar>
      <TableContainer style={{ flex: '1 1 auto' }}>
        <Table className={classes.table} stickyHeader size="small" style={{ tableLayout: 'fixed' }}>
          <TableHead>
            <TableRow>
              <TableCell className={classes.tableHeadCell}>Robot Name</TableCell>
              <TableCell className={classes.tableHeadCell}>Start Location</TableCell>
              <TableCell className={classes.tableHeadCell}>Destination</TableCell>
              <TableCell className={classes.tableHeadCell}>Active Task Duration</TableCell>
              <TableCell className={classes.tableHeadCell}>Battery</TableCell>
              <TableCell className={classes.tableHeadCell}>State</TableCell>
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
