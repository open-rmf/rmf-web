import {
  Collapse,
  makeStyles,
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  Typography,
} from '@material-ui/core';
import clsx from 'clsx';
import { formatDistanceToNow } from 'date-fns';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { rosTimeToJs } from '../utils';
import { TaskPhases } from './task-phases';
import { taskStateToStr } from './utils';
const useStyles = makeStyles((theme) => ({
  table: {
    minWidth: 650,
  },
  taskRowHover: {
    background: theme.palette.action.hover,
  },
  infoRow: {
    '& > *': {
      borderBottom: 'unset',
    },
  },
  phasesCell: {
    padding: `0 ${theme.spacing(1)}px ${theme.spacing(1)}px ${theme.spacing(1)}px`,
    boxShadow: `${theme.shadows[1]}`,
    '&:last-child': {
      paddingRight: `${theme.spacing(1)}px`,
    },
  },
  phasesRow: {
    marginBottom: theme.spacing(1),
  },
}));

interface TaskRowProps {
  task: RmfModels.TaskSummary;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

function TaskRow({ task, onClick }: TaskRowProps) {
  const classes = useStyles();
  const [hover, setHover] = React.useState(false);
  const [open, setOpen] = React.useState(false);

  function onClickFunction(ev: React.MouseEvent<HTMLTableRowElement, MouseEvent>): void {
    onClick(ev);
    setOpen(!open);
  }
  return (
    <>
      <TableRow
        className={clsx(classes.infoRow, hover && classes.taskRowHover)}
        onClick={(ev) => onClickFunction(ev)}
        onMouseOver={() => setHover(true)}
        onMouseOut={() => setHover(false)}
      >
        <TableCell>
          <Typography variant="subtitle1">{task.task_id}</Typography>
        </TableCell>
        <TableCell>
          <Typography variant="subtitle1">{task.robot_name}</Typography>
        </TableCell>
        <TableCell>
          <Typography variant="subtitle1">{toRelativeDate(task.start_time)}</Typography>
        </TableCell>
        <TableCell>
          <Typography variant="subtitle1">{toRelativeDate(task.end_time)}</Typography>
        </TableCell>
        <TableCell>
          <Typography variant="subtitle1">{taskStateToStr(task.state)}</Typography>
        </TableCell>
        {/* <TableCell>
          <IconButton aria-label="expand row" size="small" onClick={() => setOpen(!open)}>
            {open ? <KeyboardArrowUpIcon /> : <KeyboardArrowDownIcon />}
          </IconButton>
        </TableCell> */}
      </TableRow>
      <TableRow
        className={clsx(hover && classes.taskRowHover)}
        onClick={onClick}
        onMouseOver={() => setHover(true)}
        onMouseOut={() => setHover(false)}
      >
        <TableCell className={classes.phasesCell} colSpan={5}>
          <Collapse in={open} timeout="auto" unmountOnExit>
            <TaskPhases className={classes.phasesRow} taskSummary={task}></TaskPhases>
          </Collapse>
        </TableCell>
      </TableRow>
    </>
  );
}

const toRelativeDate = (rosTime: RmfModels.Time) => {
  return formatDistanceToNow(rosTimeToJs(rosTime), { addSuffix: true });
};

export interface CustomTaskTableProps {
  /**
   * The current list of tasks to display, when pagination is enabled, this should only
   * contain the tasks for the current page.
   */
  tasks: RmfModels.TaskSummary[];
  onTaskClick?(ev: React.MouseEvent<HTMLDivElement>, task: RmfModels.TaskSummary): void;
}

export function CustomTaskTable({ tasks, onTaskClick }: CustomTaskTableProps): JSX.Element {
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
