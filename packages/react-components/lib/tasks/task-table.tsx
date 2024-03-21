import {
  Button,
  styled,
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableProps,
  TableRow,
} from '@mui/material';
import { ArrowCircleDown, ArrowCircleUp } from '@mui/icons-material';
import { ApiServerModelsRmfApiTaskStateStatus as TaskStatus, TaskState } from 'api-client';
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
    backgroundColor: theme.palette.success.light,
    color: theme.palette.getContrastText(theme.palette.success.light),
  },
  [`& .${classes.taskCancelledCell}`]: {
    backgroundColor: theme.palette.grey[500],
    color: theme.palette.getContrastText(theme.palette.grey[500]),
  },
  [`& .${classes.taskCompletedCell}`]: {
    backgroundColor: theme.palette.info.light,
    color: theme.palette.getContrastText(theme.palette.info.light),
  },
  [`& .${classes.taskFailedCell}`]: {
    backgroundColor: theme.palette.error.main,
    color: theme.palette.getContrastText(theme.palette.error.main),
  },
  [`& .${classes.taskQueuedCell}`]: {
    backgroundColor: theme.palette.grey[300],
    color: theme.palette.getContrastText(theme.palette.grey[300]),
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
      case TaskStatus.Underway:
        return classes.taskActiveCell;
      case TaskStatus.Completed:
        return classes.taskCompletedCell;
      case TaskStatus.Canceled:
        return classes.taskCancelledCell;
      case TaskStatus.Failed:
        return classes.taskFailedCell;
      case TaskStatus.Queued:
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
        <TableCell>{task.category}</TableCell>
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

  /**
   * Handles the event when clicking the date title and reverts the chronological order of the list.
   * @param ev Mouse event click.
   */
  onDateTitleClick?(ev: React.MouseEvent<HTMLButtonElement>): void;

  /**
   * State: Tasks table sorting by date in descending order.
   */
  chronologicalOrder: boolean;
}

export function TaskTable({
  tasks,
  onTaskClick,
  onDateTitleClick,
  chronologicalOrder,
}: TaskTableProps): JSX.Element {
  return (
    <StyledTable stickyHeader size="small">
      <TableHead>
        <TableRow>
          <TableCell
            sx={{
              padding: 0,
            }}
          >
            <Button
              variant="text"
              onClick={(ev) => onDateTitleClick && onDateTitleClick(ev)}
              sx={{
                color: 'inherit',
                textTransform: 'none',
                width: '100%',
              }}
            >
              Date &nbsp;
              {chronologicalOrder ? <ArrowCircleDown /> : <ArrowCircleUp />}
            </Button>
          </TableCell>

          <TableCell>Task Id</TableCell>
          <TableCell>Category</TableCell>
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
