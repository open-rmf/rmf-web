import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';
import clsx from 'clsx';
import { formatDistanceToNow } from 'date-fns';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { rosTimeToJs } from '../utils';
import { taskStateToStr } from './utils';
//TO REMOVE IF TIMELINE IS USED
import Collapse from '@material-ui/core/Collapse';
import IconButton from '@material-ui/core/IconButton';
import KeyboardArrowDownIcon from '@material-ui/icons/KeyboardArrowDown';
import KeyboardArrowUpIcon from '@material-ui/icons/KeyboardArrowUp';
import { TaskPhases } from './task-phases';

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
  task: RmfModels.TaskSummary;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
  timeline: boolean;
}

function TaskRow({ task, onClick, timeline }: TaskRowProps) {
  const classes = useStyles();
  const [hover, setHover] = React.useState(false);
  const [open, setOpen] = React.useState(false);

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
        {!timeline && (
          <TableCell>
            <IconButton aria-label="expand row" size="small" onClick={() => setOpen(!open)}>
              {open ? <KeyboardArrowUpIcon /> : <KeyboardArrowDownIcon />}
            </IconButton>
          </TableCell>
        )}
      </TableRow>
      {!timeline && (
        <TableRow
          className={clsx(classes.infoRow, hover && classes.taskRowHover)}
          onClick={onClick}
          onMouseOver={() => setHover(true)}
          onMouseOut={() => setHover(false)}
        >
          <TableCell className={classes.phasesCell} colSpan={6}>
            <Collapse in={open} timeout={'auto'} unmountOnExit>
              <TaskPhases className={classes.phasesRow} taskSummary={task} />
            </Collapse>
          </TableCell>
        </TableRow>
      )}
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
  timeline: boolean;
}

export function TaskTable({ tasks, onTaskClick, timeline }: TaskTableProps): JSX.Element {
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
            timeline={timeline}
          />
        ))}
      </TableBody>
    </Table>
  );
}
