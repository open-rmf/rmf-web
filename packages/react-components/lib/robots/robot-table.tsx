import {
  Paper,
  PaperProps,
  styled,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TablePagination,
  TableRow,
  Toolbar,
  Typography,
} from '@mui/material';
import type { RobotState } from 'api-client';
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

type RobotStatus = Required<RobotState>['status'];

export interface RobotTableData {
  name: string;
  status?: RobotStatus;
  battery?: number;
  estFinishTime?: number;
}

interface RobotRowProps extends RobotTableData {
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

function getRobotStatusClass(robotStatus?: RobotStatus) {
  if (!robotStatus) {
    return '';
  }
  switch (robotStatus) {
    case 'error':
      return classes.robotErrorClass;
    case 'charging':
      return classes.robotChargingClass;
    case 'working':
      return classes.robotInMotionClass;
    case 'idle':
    case 'offline':
    case 'shutdown':
    case 'uninitialized':
      return classes.robotStoppedClass;
  }
}

const RobotRow = React.memo(
  ({ name, status, battery = 0, estFinishTime, onClick }: RobotRowProps) => {
    const robotStatusClass = getRobotStatusClass(status);

    return (
      <TableRow onClick={onClick} className={classes.tableRow}>
        <TableCell>{name}</TableCell>
        <TableCell>{estFinishTime ? new Date(estFinishTime).toLocaleString() : '-'}</TableCell>
        <TableCell>{battery * 100}%</TableCell>
        <TableCell className={robotStatusClass}>{status}</TableCell>
      </TableRow>
    );
  },
);

export type PaginationOptions = Omit<
  React.ComponentPropsWithoutRef<typeof TablePagination>,
  'component'
>;

export interface RobotTableProps extends PaperProps {
  /**
   * The current list of robots to display, when pagination is enabled, this should only
   * contain the robots for the current page.
   */
  robots: RobotTableData[];
  paginationOptions?: PaginationOptions;
  onRobotClick?(ev: React.MouseEvent<HTMLDivElement>, robotName: string): void;
}

export function RobotTable({
  robots,
  paginationOptions,
  onRobotClick,
  ...paperProps
}: RobotTableProps): JSX.Element {
  return (
    <StyledPaper {...paperProps}>
      <Toolbar>
        <Typography className={classes.title} variant="h6">
          Robots
        </Typography>
      </Toolbar>
      <TableContainer style={{ flex: '1 1 auto' }} id="robot-table">
        <Table stickyHeader size="small" style={{ tableLayout: 'fixed' }}>
          <TableHead>
            <TableRow>
              <TableCell>Robot Name</TableCell>
              <TableCell>Est. Task Finish Time</TableCell>
              <TableCell>Battery</TableCell>
              <TableCell>Status</TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            {robots.map((robot, robot_id) => (
              <RobotRow
                key={robot_id}
                {...robot}
                onClick={(ev) => onRobotClick && onRobotClick(ev, robot.name)}
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
