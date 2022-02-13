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
import type { TaskState } from 'api-client';
import { Refresh as RefreshIcon } from '@mui/icons-material';
import React from 'react';
import { VerboseRobot } from '.';

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
  verboseRobot: VerboseRobot;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

function RobotRow({ verboseRobot, onClick }: RobotRowProps) {
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
  const st = verboseRobot.state.status ? verboseRobot.state.status : '';
  const robotModeClass = getRobotModeClass(st);
  const name = verboseRobot.state.name;
  const battery = verboseRobot.state.battery ? verboseRobot.state.battery * 100 : 0;

  if (verboseRobot.current_task_state) {
    const date = verboseRobot.current_task_state.estimate_millis
      ? new Date(verboseRobot.current_task_state.estimate_millis).toISOString().substr(11, 8)
      : '-';

    return (
      <>
        <TableRow onClick={onClick} className={classes.tableRow}>
          <TableCell>{name}</TableCell>
          <TableCell>{date} </TableCell>
          <TableCell>{battery}%</TableCell>
          <TableCell className={robotModeClass}>{st}</TableCell>
        </TableRow>
      </>
    );
  } else {
    return (
      <>
        <TableRow onClick={onClick} className={classes.tableRow}>
          <TableCell>{name}</TableCell>
          <TableCell>{'-'}</TableCell>
          <TableCell>{battery}%</TableCell>
          <TableCell className={robotModeClass}>{st}</TableCell>
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
  fetchSelectedTask?: (taskId: string) => Promise<TaskState | undefined>;
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
                  verboseRobot={robot}
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
