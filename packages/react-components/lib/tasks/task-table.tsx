import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  TableProps,
  styled,
} from '@mui/material';
import clsx from 'clsx';
import { formatDistanceToNow } from 'date-fns';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { rosTimeToJs } from '../utils';
import { taskStateToStr } from './utils';

const classes = {
  table: 'task-table-table-root',
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
  [`&.${classes.table}`]: {
    minWidth: 650,
  },
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
    backgroundColor: theme.palette.primary.light,
  },
  [`& .${classes.taskCancelledCell}`]: {
    backgroundColor: theme.palette.grey[500],
  },
  [`& .${classes.taskCompletedCell}`]: {
    backgroundColor: theme.palette.success.light,
  },
  [`& .${classes.taskFailedCell}`]: {
    backgroundColor: theme.palette.error.light,
  },
  [`& .${classes.taskPendingCell}`]: {
    backgroundColor: theme.palette.info.light,
  },
  [`& .${classes.taskQueuedCell}`]: {
    backgroundColor: theme.palette.info.light,
  },
  [`& .${classes.taskUnknownCell}`]: {
    backgroundColor: theme.palette.warning.light,
  },
}));

interface TaskRowProps {
  task: RmfModels.TaskSummary;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

function TaskRow({ task, onClick }: TaskRowProps) {
  const [hover, setHover] = React.useState(false);

  const returnTaskStateCellClass = (task: RmfModels.TaskSummary) => {
    switch (task.state) {
      case RmfModels.TaskSummary.STATE_ACTIVE:
        return classes.taskActiveCell;
      case RmfModels.TaskSummary.STATE_CANCELED:
        return classes.taskCancelledCell;
      case RmfModels.TaskSummary.STATE_COMPLETED:
        return classes.taskCompletedCell;
      case RmfModels.TaskSummary.STATE_FAILED:
        return classes.taskFailedCell;
      case RmfModels.TaskSummary.STATE_PENDING:
        return classes.taskPendingCell;
      case RmfModels.TaskSummary.STATE_QUEUED:
        return classes.taskQueuedCell;
      default:
        return classes.taskUnknownCell;
    }
  };

  const taskStateCellClass = returnTaskStateCellClass(task);

  return (
    <>
      <TableRow
        className={clsx(hover && classes.taskRowHover)}
        onClick={onClick}
        onMouseOver={() => setHover(true)}
        onMouseOut={() => setHover(false)}
      >
        <TableCell>{task.task_id}</TableCell>
        <TableCell>{task.robot_name}</TableCell>
        <TableCell>{toRelativeDate(task.start_time)}</TableCell>
        <TableCell>{toRelativeDate(task.end_time)}</TableCell>
        <TableCell className={taskStateCellClass}>{taskStateToStr(task.state)}</TableCell>
      </TableRow>
    </>
  );
}

const toRelativeDate = (rosTime: RmfModels.Time) => {
  return formatDistanceToNow(rosTimeToJs(rosTime), { addSuffix: true });
};

export interface TaskTableProps {
  /**
   * The current list of tasks to display, when pagination is enabled, this should only
   * contain the tasks for the current page.
   */
  tasks: RmfModels.TaskSummary[];
  onTaskClick?(ev: React.MouseEvent<HTMLDivElement>, task: RmfModels.TaskSummary): void;
}

export function TaskTable({ tasks, onTaskClick }: TaskTableProps): JSX.Element {
  return (
    <StyledTable
      className={classes.table}
      stickyHeader
      size="small"
      style={{ tableLayout: 'fixed' }}
    >
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
            key={task.task_id}
            task={task}
            onClick={(ev) => onTaskClick && onTaskClick(ev, task)}
          />
        ))}
      </TableBody>
    </StyledTable>
  );
}
