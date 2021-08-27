import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';
import clsx from 'clsx';
import { formatDistanceToNow } from 'date-fns';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { rosTimeToJs } from '../..';
import { taskStateToStr } from '../utils';
import type { SubmitTask } from 'api-client';

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
    padding: `0 ${theme.spacing(1)}px`,
  },
  phasesRow: {
    marginBottom: theme.spacing(1),
  },
}));

export interface ScheduledTask {
  id: number;
  created_at: string;
  enabled: number[];
  rule: string;
  task_datetime: string | null;
  task_type: number;
  args?: SubmitTask;
}

interface TaskRowProps {
  scheduledTask: ScheduledTask;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

function TaskRow({ scheduledTask, onClick }: TaskRowProps) {
  const classes = useStyles();
  const [hover, setHover] = React.useState(false);

  return (
    <>
      <TableRow
        className={clsx(classes.infoRow, hover && classes.taskRowHover)}
        onClick={onClick}
        onMouseOver={() => setHover(true)}
        onMouseOut={() => setHover(false)}
      >
        <TableCell>{scheduledTask.created_at}</TableCell>
        <TableCell>{scheduledTask.enabled}</TableCell>
        <TableCell>{scheduledTask.rule}</TableCell>
        <TableCell>{scheduledTask.task_datetime}</TableCell>
        <TableCell>{scheduledTask.task_type}</TableCell>
        <TableCell>{scheduledTask.args}</TableCell>
      </TableRow>
    </>
  );
}

const toRelativeDate = (rosTime: RmfModels.Time) => {
  return formatDistanceToNow(rosTimeToJs(rosTime), { addSuffix: true });
};

export interface ScheduledTaskTableProps {
  /**
   * The current list of tasks to display, when pagination is enabled, this should only
   * contain the tasks for the current page.
   */
  taskRules: ScheduledTask[];
  onTaskClick?(ev: React.MouseEvent<HTMLDivElement>, taskRule: ScheduledTask): void;
}

export function ScheduledTaskTable(props: ScheduledTaskTableProps): JSX.Element {
  const { taskRules, onTaskClick } = props;
  const classes = useStyles();
  return (
    <Table className={classes.table} stickyHeader size="small" style={{ tableLayout: 'fixed' }}>
      <TableHead>
        <TableRow>
          <TableCell>Rule Name</TableCell>
          <TableCell>Task</TableCell>
          <TableCell>Frequency</TableCell>
          <TableCell>Frequency Type</TableCell>
          <TableCell>Start Time</TableCell>
          <TableCell>End Time</TableCell>
          <TableCell>Days of week</TableCell>
          <TableCell>Created at</TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {taskRules.map((task) => (
          <TaskRow
            key={task.id}
            scheduledTask={task}
            onClick={(ev) => onTaskClick && onTaskClick(ev, task)}
          />
        ))}
      </TableBody>
    </Table>
  );
}
