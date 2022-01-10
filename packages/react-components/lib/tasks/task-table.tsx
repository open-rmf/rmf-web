import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  TableProps,
  styled,
} from '@mui/material';
import type { TaskState, Time } from 'api-client';
import clsx from 'clsx';
import { formatDistanceToNow, format } from 'date-fns';
import React from 'react';
import { rosTimeToJs } from '../utils';
import { getState } from './utils';

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
  [`& .${classes.taskPendingCell}`]: {
    backgroundColor: theme.palette.info.dark,
    color: theme.palette.getContrastText(theme.palette.info.light),
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

  const returnTaskStateCellClass = (task: TaskState) => {
    if (getState(task) === 'Underway') return classes.taskActiveCell;
    if (getState(task) === 'Completed') return classes.taskCompletedCell;
    return classes.taskUnknownCell;
  };

  const taskStateCellClass = returnTaskStateCellClass(task);
  // TODO - replace robot name with something else
  return (
    <>
      <TableRow
        className={clsx(hover && classes.taskRowHover)}
        onClick={onClick}
        onMouseOver={() => setHover(true)}
        onMouseOut={() => setHover(false)}
      >
        <TableCell>{task.booking.id}</TableCell>
        <TableCell>{'robotname'}</TableCell>
        <TableCell>
          {task.unix_millis_start_time
            ? format(new Date(task.unix_millis_start_time * 1000), 'dd - mm - yyyy')
            : '-'}
        </TableCell>
        <TableCell>
          {task.unix_millis_finish_time
            ? format(new Date(task.unix_millis_finish_time * 1000), 'dd - mm - yyyy')
            : '-'}
        </TableCell>
        <TableCell className={taskStateCellClass}>{task ? getState(task) : ''}</TableCell>
      </TableRow>
    </>
  );
}

const toRelativeDate = (rosTime: Time) => {
  return formatDistanceToNow(rosTimeToJs(rosTime), { addSuffix: true });
};

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
