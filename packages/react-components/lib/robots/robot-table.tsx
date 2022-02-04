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
} from '@mui/material';
import type { RobotState, TaskState } from 'api-client';
import { Refresh as RefreshIcon } from '@mui/icons-material';
import React from 'react';

const classes = {
  table: 'robot-table',
  title: 'robot-table-title',
  infoRow: 'robot-table-info-row',
  phasesCell: 'robot-table-phases-cell',
  robotErrorClass: 'robot-table-error',
  robotStoppedClass: 'robot-table-stopped',
  robotInMotionClass: 'robot-table-in-motion',
  robotChargingClass: 'robot-table-charging',
  tableRow: 'robot-table-row-hover',
};
const StyledPaper = styled((props: PaperProps) => <Paper {...props} />)(({ theme }) => ({
  [`& .${classes.table}`]: {
    minWidth: 650,
  },
  [`& .${classes.title}`]: {
    flex: '1 1 100%',
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
  [`& .${classes.tableRow}`]: {
    '&:hover': {
      cursor: 'pointer',
      backgroundColor: theme.palette.action.hover,
    },
  },
}));

interface RobotRowProps {
  robot: RobotState;
  fetchSelectedTask?: (taskId: string) => Promise<TaskState | undefined>;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

function RobotRow({ robot, fetchSelectedTask, onClick }: RobotRowProps) {
  const getRobotModeClass = (robotMode: string) => {
    switch (robotMode) {
      case 'emergency':
        return classes.robotErrorClass;
      case 'charging':
        return classes.robotChargingClass;
      case 'working':
        return classes.robotInMotionClass;
      case 'idle':
      case 'paused':
      case 'waiting':
        return classes.robotStoppedClass;
      default:
        return '';
    }
  };
  let robotModeClass = '';
  if (robot.status) robotModeClass = getRobotModeClass(robot.status);

  const [currentTask, setCurrentTask] = React.useState<TaskState | undefined>();
  React.useEffect(() => {
    (async () => {
      if (robot.task_id) {
        fetchSelectedTask && setCurrentTask(await fetchSelectedTask(robot.task_id));
      }
    })();
  });

  if (robot.task_id) {
    return (
      <>
        <TableRow onClick={onClick} className={classes.tableRow}>
          <TableCell>{robot.name}</TableCell>
          <TableCell>
            {currentTask && currentTask.estimate_millis
              ? new Date(currentTask.estimate_millis).toISOString().substr(11, 8)
              : '-'}
          </TableCell>
          <TableCell>{robot.battery ? robot.battery * 100 : 0}%</TableCell>
          <TableCell className={robotModeClass}>{robot.status}</TableCell>
        </TableRow>
      </>
    );
  } else {
    return (
      <>
        <TableRow onClick={onClick} className={classes.tableRow}>
          <TableCell>{robot.name}</TableCell>
          <TableCell>{'-'}</TableCell>
          <TableCell>{robot.battery ? robot.battery * 100 : 0}%</TableCell>
          <TableCell className={robotModeClass}>{robot.status}</TableCell>
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
  robots: RobotState[];
  fetchSelectedTask?: (taskId: string) => Promise<TaskState | undefined>;
  paginationOptions?: PaginationOptions;
  onRefreshClick?: React.MouseEventHandler<HTMLButtonElement>;
  onRobotClick?(ev: React.MouseEvent<HTMLDivElement>, robot: RobotState): void;
}

export function RobotTable({
  robots,
  fetchSelectedTask,
  paginationOptions,
  onRefreshClick,
  onRobotClick,
  ...paperProps
}: RobotTableProps): JSX.Element {
  return (
    <StyledPaper {...paperProps}>
      <Toolbar>
        <Typography className={classes.title} variant="h6">
          Robots
        </Typography>
        <IconButton onClick={onRefreshClick} aria-label="Refresh">
          <RefreshIcon />
        </IconButton>
      </Toolbar>
      <TableContainer style={{ flex: '1 1 auto' }} id="robot-table">
        <Table stickyHeader size="small" style={{ tableLayout: 'fixed' }}>
          <TableHead>
            <TableRow>
              <TableCell>Robot Name</TableCell>
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
                  fetchSelectedTask={fetchSelectedTask}
                  onClick={(ev) => onRobotClick && onRobotClick(ev, robot)}
                />
              ))}
          </TableBody>
        </Table>
      </TableContainer>
      {paginationOptions && (
        <TablePagination component="div" {...paginationOptions} style={{ flex: '0 0 auto' }} />
      )}
    </StyledPaper>
  );
}
