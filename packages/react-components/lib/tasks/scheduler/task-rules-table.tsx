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

export interface TaskRule {
  id: number;
  name: string;
  days_of_week: number[];
  start_datetime: string;
  end_datetime: string | null;
  frequency: number;
  frequency_type: string;
  task_type: string;
  args?: SubmitTask;
}

interface TaskRowProps {
  taskRule: TaskRule;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

function TaskRow({ taskRule, onClick }: TaskRowProps) {
  const classes = useStyles();
  const [hover, setHover] = React.useState(false);

  const parsedDayOfWeek = (daysOfWeek: any): string | null => {
    if (!daysOfWeek) return null;
    const days = [];
    for (const [key, value] of Object.entries(daysOfWeek)) {
      if (value === true) {
        days.push(key);
      }
    }
    return days.join(', ');
  };
  return (
    <>
      <TableRow
        className={clsx(classes.infoRow, hover && classes.taskRowHover)}
        onClick={onClick}
        onMouseOver={() => setHover(true)}
        onMouseOut={() => setHover(false)}
      >
        <TableCell>{taskRule.name}</TableCell>
        <TableCell>{taskRule.frequency}</TableCell>
        <TableCell>{taskRule.frequency_type}</TableCell>
        <TableCell>{taskRule.start_datetime}</TableCell>
        <TableCell>{taskRule.end_datetime ? taskRule.end_datetime : null}</TableCell>
        <TableCell>{taskRule.days_of_week && parsedDayOfWeek(taskRule.days_of_week)}</TableCell>
        <TableCell>{JSON.stringify(taskRule.args?.description)}</TableCell>
      </TableRow>
    </>
  );
}

const toRelativeDate = (rosTime: RmfModels.Time) => {
  return formatDistanceToNow(rosTimeToJs(rosTime), { addSuffix: true });
};

export interface TaskRuleTableProps {
  /**
   * The current list of tasks to display, when pagination is enabled, this should only
   * contain the tasks for the current page.
   */
  taskRules: TaskRule[];
  onTaskClick?(ev: React.MouseEvent<HTMLDivElement>, taskRule: TaskRule): void;
  onLoad?(): Promise<void>;
}

export function TaskRuleTable(props: TaskRuleTableProps): JSX.Element {
  const { taskRules, onTaskClick, onLoad } = props;
  const classes = useStyles();

  React.useEffect(() => {
    onLoad && onLoad();
  }, [onLoad]);

  return (
    <Table className={classes.table} stickyHeader size="small" style={{ tableLayout: 'fixed' }}>
      <TableHead>
        <TableRow>
          <TableCell>Rule Name</TableCell>
          <TableCell>Frequency</TableCell>
          <TableCell>Frequency Type</TableCell>
          <TableCell>Start Time</TableCell>
          <TableCell>End Time</TableCell>
          <TableCell>Days of week</TableCell>
          <TableCell>Task</TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {taskRules.map((task) => (
          <TaskRow
            key={task.id}
            taskRule={task}
            onClick={(ev) => onTaskClick && onTaskClick(ev, task)}
          />
        ))}
      </TableBody>
    </Table>
  );
}

export default TaskRuleTable;
