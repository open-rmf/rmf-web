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
  enabled: boolean;
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
        <TableCell>{scheduledTask.enabled}</TableCell>
        {/* <TableCell>{scheduledTask.rule}</TableCell> */}
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
  scheduledTasks: ScheduledTask[];
  onTaskClick?(ev: React.MouseEvent<HTMLDivElement>, task: ScheduledTask): void;
  onLoad?(): Promise<void>;
}

export function ScheduledTaskTable(props: ScheduledTaskTableProps): JSX.Element {
  const { scheduledTasks, onTaskClick, onLoad } = props;
  const classes = useStyles();

  React.useEffect(() => {
    onLoad && onLoad();
  }, [onLoad]);

  return (
    <Table className={classes.table} stickyHeader size="small" style={{ tableLayout: 'fixed' }}>
      <TableHead>
        <TableRow>
          <TableCell>Enabled</TableCell>
          <TableCell>Start Time</TableCell>
          <TableCell>Task Type</TableCell>
          <TableCell>Task (Args)</TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {scheduledTasks.map((task) => (
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
