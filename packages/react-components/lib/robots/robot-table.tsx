import {
  IconButton,
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
  styled,
} from '@material-ui/core';
import * as RmfModels from 'rmf-models';
import { Refresh as RefreshIcon } from '@material-ui/icons';
import React from 'react';
import { taskTypeToStr } from '../tasks/utils';
import { robotModeToString, VerboseRobot } from './utils';

const classes = {
  table: 'robot-table',
  title: 'robot-table-title',
  infoRow: 'robot-table-info-row',
  taskRowHover: 'robot-table-row-hover',
  phasesCell: 'robot-table-phases-cell',
  robotErrorClass: 'robot-table-error',
  robotStoppedClass: 'robot-table-stopped',
  robotInMotionClass: 'robot-table-in-motion',
  robotChargingClass: 'robot-table-charging',
};
const RobotTableRoot = styled((props: PaperProps) => <Paper {...props} />)(({ theme }) => ({
  [`& .${classes.table}`]: {
    minWidth: 650,
  },
  [`& .${classes.title}`]: {
    flex: '1 1 100%',
  },
  [`& .${classes.taskRowHover}`]: {
    background: theme.palette.action.hover,
  },
  [`& .${classes.phasesCell}`]: {
    padding: `0 ${theme.spacing(1)}px`,
  },
  [`& .${classes.robotErrorClass}`]: {
    backgroundColor: theme.palette.error.main,
  },
  [`& .${classes.robotStoppedClass}`]: {
    backgroundColor: theme.palette.warning.main,
  },
  [`& .${classes.robotInMotionClass}`]: {
    backgroundColor: theme.palette.success.main,
  },
  [`& .${classes.robotChargingClass}`]: {
    backgroundColor: theme.palette.info.main,
  },
}));

interface RobotRowProps {
  robot: VerboseRobot;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

const returnLocationCells = (robot: VerboseRobot) => {
  const taskDescription = robot.tasks[0].summary.task_profile.description;
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
  const getRobotModeClass = (robotMode: RmfModels.RobotMode) => {
    switch (robotMode.mode) {
      case RmfModels.RobotMode.MODE_EMERGENCY:
        return classes.robotErrorClass;
      case RmfModels.RobotMode.MODE_CHARGING:
        return classes.robotChargingClass;
      case RmfModels.RobotMode.MODE_GOING_HOME:
      case RmfModels.RobotMode.MODE_DOCKING:
      case RmfModels.RobotMode.MODE_MOVING:
        return classes.robotInMotionClass;
      case RmfModels.RobotMode.MODE_IDLE:
      case RmfModels.RobotMode.MODE_PAUSED:
      case RmfModels.RobotMode.MODE_WAITING:
        return classes.robotStoppedClass;
      default:
        return '';
    }
  };

  const robotMode = robotModeToString(robot.state.mode);
  const robotModeClass = getRobotModeClass(robot.state.mode);

  if (robot.tasks.length === 0) {
    return (
      <>
        <TableRow onClick={onClick}>
          <TableCell>{robot.name}</TableCell>
          <TableCell>{'-'}</TableCell>
          <TableCell>{'-'}</TableCell>
          <TableCell>{'-'}</TableCell>
          <TableCell>{robot.state.battery_percent.toFixed(2)}%</TableCell>
          <TableCell className={robotModeClass}>{robotMode}</TableCell>
        </TableRow>
      </>
    );
  } else {
    return (
      <>
        <TableRow onClick={onClick}>
          <TableCell>{robot.name}</TableCell>
          {returnLocationCells(robot)}
          <TableCell>
            {robot.tasks
              ? robot.tasks[0].summary.end_time.sec - robot.tasks[0].summary.start_time.sec
              : '-'}
          </TableCell>
          <TableCell>{robot.state.battery_percent.toFixed(2)}%</TableCell>
          <TableCell className={robotModeClass}>{robotMode}</TableCell>
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
  robots: VerboseRobot[];
  paginationOptions?: PaginationOptions;
  onRefreshClick?: React.MouseEventHandler<HTMLButtonElement>;
  onRobotClick?(ev: React.MouseEvent<HTMLDivElement>, robot: VerboseRobot): void;
}

export function RobotTable({
  robots,
  paginationOptions,
  onRefreshClick,
  onRobotClick,
  ...paperProps
}: RobotTableProps): JSX.Element {
  return (
    <RobotTableRoot {...paperProps}>
      <Toolbar>
        <Typography className={classes.title} variant="h6">
          Robots
        </Typography>
        <IconButton onClick={onRefreshClick} aria-label="Refresh">
          <RefreshIcon />
        </IconButton>
      </Toolbar>
      <TableContainer style={{ flex: '1 1 auto' }} id="robot-table">
        <Table className={classes.table} stickyHeader size="small" style={{ tableLayout: 'fixed' }}>
          <TableHead>
            <TableRow>
              <TableCell>Robot Name</TableCell>
              <TableCell>Start Location</TableCell>
              <TableCell>Destination</TableCell>
              <TableCell>Active Task Duration</TableCell>
              <TableCell>Battery</TableCell>
              <TableCell>State</TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            {robots &&
              robots.map((robot, robot_id) => (
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
    </RobotTableRoot>
  );
}
