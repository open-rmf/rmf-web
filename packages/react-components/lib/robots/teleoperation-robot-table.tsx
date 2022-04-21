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

export interface TeleoperationRobotConfig {
  name: string;
  id: string;
  accessKey: string;
}

interface RobotRowProps extends TeleoperationRobotConfig {
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

const RobotRow = React.memo(({ name, onClick }: RobotRowProps) => {
  return (
    <TableRow onClick={onClick} className={classes.tableRow}>
      <TableCell>{name}</TableCell>
    </TableRow>
  );
});

type PaginationOptions = Omit<React.ComponentPropsWithoutRef<typeof TablePagination>, 'component'>;

export interface TeleoperationRobotTableProps extends PaperProps {
  /**
   * The current list of robots to display, when pagination is enabled, this should only
   * contain the robots for the current page.
   */
  robots: TeleoperationRobotConfig[];
  paginationOptions?: PaginationOptions;
  onRobotClick?(ev: React.MouseEvent<HTMLDivElement>, robotName: string): void;
}

export function TeleoperationRobotTable({
  robots,
  paginationOptions,
  onRobotClick,
  ...paperProps
}: TeleoperationRobotTableProps): JSX.Element {
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
            </TableRow>
          </TableHead>
          <TableBody>
            {robots.map((robot, id) => (
              <RobotRow
                key={id}
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
