import {
  SxProps,
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableProps,
  TableRow,
  useTheme,
} from '@mui/material';
import type { RobotState } from 'api-client';
import React from 'react';

type RobotStatus = Required<RobotState>['status'];

export interface RobotTableData {
  fleet: string;
  name: string;
  status?: RobotStatus;
  battery?: number;
  estFinishTime?: number;
  lastUpdateTime?: number;
  level?: string;
}

interface RobotRowProps extends RobotTableData {
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

const RobotRow = React.memo(
  ({ fleet, name, status, battery = 0, estFinishTime, lastUpdateTime, onClick }: RobotRowProps) => {
    const theme = useTheme();

    const robotStatusClass: SxProps = React.useMemo(() => {
      if (!status) {
        return {};
      }
      switch (status) {
        case 'error':
          return {
            backgroundColor: theme.palette.error.main,
          };
        case 'charging':
          return {
            backgroundColor: theme.palette.info.main,
          };
        case 'working':
          return {
            backgroundColor: theme.palette.success.main,
          };
        case 'idle':
        case 'offline':
        case 'shutdown':
        case 'uninitialized':
          return {
            backgroundColor: theme.palette.warning.main,
          };
      }
    }, [status, theme]);

    return (
      <TableRow
        onClick={onClick}
        sx={{
          cursor: 'pointer',
          backgroundColor: theme.palette.action.hover,
        }}
      >
        <TableCell>{fleet}</TableCell>
        <TableCell>{name}</TableCell>
        <TableCell>{estFinishTime ? new Date(estFinishTime).toLocaleString() : '-'}</TableCell>
        <TableCell>{(battery * 100).toFixed(2)}%</TableCell>
        <TableCell>{lastUpdateTime ? new Date(lastUpdateTime).toLocaleString() : '-'}</TableCell>
        <TableCell sx={robotStatusClass}>{status}</TableCell>
      </TableRow>
    );
  },
);

export interface RobotTableProps extends TableProps {
  /**
   * The current list of robots to display, when pagination is enabled, this should only
   * contain the robots for the current page.
   */
  robots: RobotTableData[];
  onRobotClick?(ev: React.MouseEvent<HTMLDivElement>, robotName: RobotTableData): void;
}

export function RobotTable({ robots, onRobotClick, ...otherProps }: RobotTableProps): JSX.Element {
  return (
    <Table stickyHeader size="small" style={{ tableLayout: 'fixed' }} {...otherProps}>
      <TableHead>
        <TableRow>
          <TableCell>Fleet</TableCell>
          <TableCell>Robot Name</TableCell>
          <TableCell>Est. Task Finish Time</TableCell>
          <TableCell>Battery</TableCell>
          <TableCell>Last Updated</TableCell>
          <TableCell>Status</TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {robots.map((robot, robot_id) => (
          <RobotRow
            key={robot_id}
            {...robot}
            onClick={(ev) => onRobotClick && onRobotClick(ev, robot)}
          />
        ))}
      </TableBody>
    </Table>
  );
}
