import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';
import type { TaskSummary, Time } from 'api-client';
import clsx from 'clsx';
import { formatDistanceToNow } from 'date-fns';
import React from 'react';
import { TaskSummary as RmfTaskSummary } from 'rmf-models';
import { rosTimeToJs } from '../utils';
import { taskStateToStr } from './utils';

const useStyles = makeStyles((theme) => ({
  table: {
    minWidth: 650,
  },
  taskRowHover: {
    background: theme.palette.action.hover,
    cursor: 'pointer',
  },
  infoRow: {
    '& > *': {
      borderBottom: 'unset',
    },
  },
  phasesCell: {
    padding: `0 ${theme.spacing(1)}px 0 ${theme.spacing(1)}px`,
    boxShadow: `${theme.shadows[1]}`,
    '&:last-child': {
      paddingRight: `${theme.spacing(1)}px`,
    },
  },
  phasesRow: {
    marginBottom: theme.spacing(1),
    marginTop: theme.spacing(1),
  },
  taskActiveCell: {
    backgroundColor: theme.palette.primary.light,
  },
  taskCancelledCell: {
    backgroundColor: theme.palette.grey[500],
  },
  taskCompletedCell: {
    backgroundColor: theme.palette.success.light,
  },
  taskFailedCell: {
    backgroundColor: theme.palette.error.light,
  },
  taskPendingCell: {
    backgroundColor: theme.palette.info.light,
  },
  taskQueuedCell: {
    backgroundColor: theme.palette.info.light,
  },
  taskUnknownCell: {
    backgroundColor: theme.palette.warning.light,
  },
}));

interface TaskRowProps {
  task: TaskSummary;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

function TaskRow({ task, onClick }: TaskRowProps) {
  const classes = useStyles();
  const [hover, setHover] = React.useState(false);

  const returnTaskStateCellClass = (task: TaskSummary) => {
    switch (task.state) {
      case RmfTaskSummary.STATE_ACTIVE:
        return classes.taskActiveCell;
      case RmfTaskSummary.STATE_CANCELED:
        return classes.taskCancelledCell;
      case RmfTaskSummary.STATE_COMPLETED:
        return classes.taskCompletedCell;
      case RmfTaskSummary.STATE_FAILED:
        return classes.taskFailedCell;
      case RmfTaskSummary.STATE_PENDING:
        return classes.taskPendingCell;
      case RmfTaskSummary.STATE_QUEUED:
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

const toRelativeDate = (rosTime: Time) => {
  return formatDistanceToNow(rosTimeToJs(rosTime), { addSuffix: true });
};

export interface TaskTableProps {
  /**
   * The current list of tasks to display, when pagination is enabled, this should only
   * contain the tasks for the current page.
   */
  tasks: TaskSummary[];
  onTaskClick?(ev: React.MouseEvent<HTMLDivElement>, task: TaskSummary): void;
}

export function TaskTable({ tasks, onTaskClick }: TaskTableProps): JSX.Element {
  const classes = useStyles();
  return (
    <Table className={classes.table} stickyHeader size="small" style={{ tableLayout: 'fixed' }}>
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
    </Table>
  );
}
