import {
  styled,
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableProps,
  TableRow,
} from '@mui/material';
import { Status, TaskState } from 'api-client';
import clsx from 'clsx';
import React from 'react';

const classes = {
  taskRowHover: 'task-table-taskrow-hover',
  infoRow: 'task-table-info-row',
  phasesCell: 'task-table-phase-cell',
  phasesRow: 'task-table-phase-row',
  taskActiveCell: 'task-table-active-cell',
  taskCancelledCell: 'task-table-cancelled-cell',
  taskCompletedCell: 'task-table-completed-cell',
  taskFailedCell: 'task-table-failed-cell',
  taskPendingCell: 'task-table-pending-cell',
  taskQueuedCell: 'task-table-queued-cell',
  taskUnknownCell: 'task-table-unknown-cell',
};
const StyledTable = styled((props: TableProps) => <Table {...props} />)(({ theme }) => ({
  [`& .${classes.taskRowHover}`]: {
    background: theme.palette.action.hover,
    cursor: 'pointer',
  },
  [`& .${classes.infoRow}`]: {
    '& > *': {
      borderBottom: 'unset',
    },
  },
  [`& .${classes.phasesCell}`]: {
    padding: `0 ${theme.spacing(1)}px 0 ${theme.spacing(1)}px`,
    boxShadow: `${theme.shadows[1]}`,
    '&:last-child': {
      paddingRight: `${theme.spacing(1)}px`,
    },
  },
  [`& .${classes.phasesRow}`]: {
    marginBottom: theme.spacing(1),
    marginTop: theme.spacing(1),
  },
  [`& .${classes.taskActiveCell}`]: {
    backgroundColor: theme.palette.info.light,
    color: theme.palette.getContrastText(theme.palette.info.light),
  },
  [`& .${classes.taskCancelledCell}`]: {
    backgroundColor: theme.palette.grey[500],
  },
  [`& .${classes.taskCompletedCell}`]: {
    backgroundColor: theme.palette.success.main,
    color: theme.palette.getContrastText(theme.palette.success.main),
  },
  [`& .${classes.taskFailedCell}`]: {
    backgroundColor: theme.palette.error.main,
    color: theme.palette.getContrastText(theme.palette.error.main),
  },
  [`& .${classes.taskQueuedCell}`]: {
    backgroundColor: theme.palette.info.dark,
    color: theme.palette.getContrastText(theme.palette.info.light),
  },
  [`& .${classes.taskUnknownCell}`]: {
    backgroundColor: theme.palette.warning.main,
    color: theme.palette.getContrastText(theme.palette.warning.main),
  },
}));

interface TaskRowProps {
  task: TaskState;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

function TaskRow({ task, onClick }: TaskRowProps) {
  // replace all temp info
  const [hover, setHover] = React.useState(false);

  const getTaskStateCellClass = (task: TaskState) => {
    switch (task.status) {
      case Status.Underway:
        return classes.taskActiveCell;
      case Status.Completed:
        return classes.taskCompletedCell;
      case Status.Canceled:
        return classes.taskCancelledCell;
      case Status.Failed:
        return classes.taskFailedCell;
      case Status.Queued:
        return classes.taskQueuedCell;
      default:
        return classes.taskUnknownCell;
    }
  };

  const taskStateCellClass = getTaskStateCellClass(task);
  return (
    <>
      <TableRow
        className={clsx(hover && classes.taskRowHover)}
        onClick={onClick}
        onMouseOver={() => setHover(true)}
        onMouseOut={() => setHover(false)}
      >
        <TableCell>
          {task.unix_millis_start_time
            ? new Date(task.unix_millis_start_time).toLocaleDateString()
            : 'unknown'}
        </TableCell>
        <TableCell>{task.booking.id}</TableCell>
        <TableCell>{task.assigned_to ? task.assigned_to.name : 'unknown'}</TableCell>
        <TableCell>
          {task.unix_millis_start_time
            ? new Date(task.unix_millis_start_time).toLocaleTimeString()
            : '-'}
        </TableCell>
        <TableCell>
          {task.unix_millis_finish_time
            ? new Date(task.unix_millis_finish_time).toLocaleTimeString()
            : '-'}
        </TableCell>
        <TableCell className={taskStateCellClass}>{task.status || 'unknown'}</TableCell>
      </TableRow>
    </>
  );
}

export interface TaskTableProps {
  /**
   * The current list of tasks to display, when pagination is enabled, this should only
   * contain the tasks for the current page.
   */
  tasks: TaskState[];
  onTaskClick?(ev: React.MouseEvent<HTMLDivElement>, task: TaskState): void;
}

export function TaskTable({ tasks, onTaskClick }: TaskTableProps): JSX.Element {
  return (
    <StyledTable stickyHeader size="small">
      <TableHead>
        <TableRow>
          <TableCell>Date</TableCell>
          <TableCell>Task Id</TableCell>
          <TableCell>Assignee</TableCell>
          <TableCell>Start Time</TableCell>
          <TableCell>End Time</TableCell>
          <TableCell>State</TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {tasks.map((task) => (
          <TaskRow
            key={task.booking.id}
            task={task}
            onClick={(ev) => onTaskClick && onTaskClick(ev, task)}
          />
        ))}
      </TableBody>
    </StyledTable>
  );
}
