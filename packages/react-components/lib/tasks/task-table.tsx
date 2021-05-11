import {
  IconButton,
  makeStyles,
  Paper,
  PaperProps,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TablePagination,
  TableRow,
  Toolbar,
  Typography,
  withStyles,
} from '@material-ui/core';
import { AddOutlined as AddOutlinedIcon, Refresh as RefreshIcon } from '@material-ui/icons';
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
  title: {
    flex: '1 1 100%',
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
    borderBottom: 'none',
  },
  tablePagination: {
    flex: '0 0 auto',
    borderBottom: 'none',
  },
}));

const TablePaginationTheme = withStyles({
  actions: {
    '& .Mui-disabled': {
      color: '#A8A8A8',
    },
  },
})(TablePagination);

interface TaskRowProps {
  task: RmfModels.TaskSummary;
  componentTheme?: string;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

function TaskRow({ task, onClick, componentTheme }: TaskRowProps) {
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
        <TableCell className={componentTheme}>{task.task_id}</TableCell>
        <TableCell className={componentTheme}>{task.robot_name}</TableCell>
        <TableCell className={componentTheme}>{toRelativeDate(task.start_time)}</TableCell>
        <TableCell className={componentTheme}>{toRelativeDate(task.end_time)}</TableCell>
        <TableCell className={componentTheme}>{taskStateToStr(task.state)}</TableCell>
      </TableRow>
      <TableRow
        className={clsx(hover && classes.taskRowHover)}
        onClick={onClick}
        onMouseOver={() => setHover(true)}
        onMouseOut={() => setHover(false)}
      >
        <TableCell className={classes.phasesCell} colSpan={5}>
          <TaskPhases taskSummary={task}></TaskPhases>
        </TableCell>
      </TableRow>
    </>
  );
}

const toRelativeDate = (rosTime: RmfModels.Time) => {
  return formatDistanceToNow(rosTimeToJs(rosTime), { addSuffix: true });
};

export type PaginationOptions = Omit<
  React.ComponentPropsWithoutRef<typeof TablePagination>,
  'component'
>;

export interface TaskTableProps extends PaperProps {
  /**
   * The current list of tasks to display, when pagination is enabled, this should only
   * contain the tasks for the current page.
   */
  tasks: RmfModels.TaskSummary[];
  paginationOptions?: PaginationOptions;
  componentTheme?: string;
  onCreateTaskClick?: React.MouseEventHandler<HTMLButtonElement>;
  onTaskClick?(ev: React.MouseEvent<HTMLDivElement>, task: RmfModels.TaskSummary): void;
  onRefreshClick?: React.MouseEventHandler<HTMLButtonElement>;
}

export function TaskTable({
  tasks,
  paginationOptions,
  onCreateTaskClick,
  onTaskClick,
  onRefreshClick,
  componentTheme,
  ...paperProps
}: TaskTableProps): JSX.Element {
  const classes = useStyles();
  return (
    <Paper {...paperProps}>
      <Toolbar>
        <Typography className={classes.title} variant="h6">
          Tasks
        </Typography>
        <IconButton onClick={onRefreshClick} aria-label="Refresh">
          <RefreshIcon className={componentTheme} />
        </IconButton>
        <IconButton onClick={onCreateTaskClick} aria-label="Create Task">
          <AddOutlinedIcon className={componentTheme} />
        </IconButton>
      </Toolbar>
      <TableContainer style={{ flex: '1 1 auto' }}>
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
                componentTheme={componentTheme}
              />
            ))}
          </TableBody>
        </Table>
      </TableContainer>
      {paginationOptions && (
        <TablePaginationTheme
          {...paginationOptions}
          className={`${classes.tablePagination} ${componentTheme}`}
        />
      )}
    </Paper>
  );
}
