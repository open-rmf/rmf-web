import {
  Grid,
  IconButton,
  makeStyles,
  Paper,
  Snackbar,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TablePagination,
  TableRow,
  Toolbar,
  Typography,
} from '@material-ui/core';
import { AddOutlined as AddOutlinedIcon, Refresh as RefreshIcon } from '@material-ui/icons';
import { Alert, AlertProps } from '@material-ui/lab';
import clsx from 'clsx';
import { formatDistanceToNow } from 'date-fns';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { rosTimeToJs } from '../utils';
import { CreateTaskForm, CreateTaskFormProps } from './create-task';
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
  },
}));

interface TaskRowProps {
  task: RmfModels.TaskSummary;
  onClick: React.MouseEventHandler<HTMLTableRowElement>;
}

function TaskRow({ task, onClick }: TaskRowProps) {
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
        <TableCell>{task.task_id}</TableCell>
        <TableCell>{task.robot_name}</TableCell>
        <TableCell>{toRelativeDate(task.start_time)}</TableCell>
        <TableCell>{toRelativeDate(task.end_time)}</TableCell>
        <TableCell>{taskStateToStr(task.state)}</TableCell>
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

export interface TaskTableProps {
  /**
   * The current list of tasks to display, when pagination is enabled, this should only
   * contain the tasks for the current page.
   */
  tasks: RmfModels.TaskSummary[];
  paginationOptions?: PaginationOptions;
  submitTask?: CreateTaskFormProps['submitTask'];
  onTaskClick?(ev: React.MouseEvent<HTMLDivElement>, task: RmfModels.TaskSummary): void;
  onRefreshClick?: React.MouseEventHandler<HTMLButtonElement>;
}

export function TaskTable({
  tasks,
  paginationOptions,
  submitTask,
  onTaskClick,
  onRefreshClick,
}: TaskTableProps): JSX.Element {
  const classes = useStyles();
  const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
  const [openSubmitResultSnackbar, setOpenSubmitResultSnackbar] = React.useState(false);
  const [snackbarMessage, setSnackbarMessage] = React.useState('');
  const [snackbarSeverity, setSnackbarSeverity] = React.useState<AlertProps['severity']>('success');

  return (
    <>
      <Paper style={{ height: '100%' }}>
        <Grid container direction="column" wrap="nowrap" style={{ height: 'inherit' }}>
          <Toolbar>
            <Typography className={classes.title} variant="h6">
              Tasks
            </Typography>
            <IconButton onClick={onRefreshClick} aria-label="Refresh">
              <RefreshIcon />
            </IconButton>
            <IconButton onClick={() => setOpenCreateTaskForm(true)} aria-label="Create Task">
              <AddOutlinedIcon />
            </IconButton>
          </Toolbar>
          <TableContainer style={{ flex: '1 1 auto' }}>
            <Table
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
            </Table>
          </TableContainer>
          {paginationOptions && (
            <TablePagination component="div" {...paginationOptions} style={{ flex: '0 0 auto' }} />
          )}
        </Grid>
      </Paper>
      <CreateTaskForm
        open={openCreateTaskForm}
        onClose={() => setOpenCreateTaskForm(false)}
        submitTask={submitTask}
        onCancelClick={() => setOpenCreateTaskForm(false)}
        onSuccess={() => {
          setOpenCreateTaskForm(false);
          setSnackbarSeverity('success');
          setSnackbarMessage('Successfully created task');
          setOpenSubmitResultSnackbar(true);
        }}
        onFail={(e) => {
          setSnackbarSeverity('error');
          setSnackbarMessage(`Failed to create task: ${e.message}`);
          setOpenSubmitResultSnackbar(true);
        }}
      />
      <Snackbar
        open={openSubmitResultSnackbar}
        onClose={() => setOpenSubmitResultSnackbar(false)}
        autoHideDuration={2000}
      >
        <Alert severity={snackbarSeverity}>{snackbarMessage}</Alert>
      </Snackbar>
    </>
  );
}
