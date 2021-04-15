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
import AddOutlinedIcon from '@material-ui/icons/AddCircle';
import { Alert, AlertProps } from '@material-ui/lab';
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
  infoRow: {
    '& > *': {
      borderBottom: 'unset',
    },
  },
  phasesCell: {
    padding: `0 ${theme.spacing(1)}px`,
  },
}));

interface TableToolbarProps {
  onCreateTaskClick?: React.MouseEventHandler<HTMLButtonElement>;
}

function TableToolbar({ onCreateTaskClick }: TableToolbarProps) {
  const classes = useStyles();
  return (
    <Toolbar>
      <Typography className={classes.title} variant="h6">
        Tasks
      </Typography>
      <IconButton onClick={onCreateTaskClick} aria-label="Create Task">
        <AddOutlinedIcon />
      </IconButton>
    </Toolbar>
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
}

export function TaskTable({ tasks, paginationOptions, submitTask }: TaskTableProps): JSX.Element {
  const classes = useStyles();
  const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
  const [openSubmitResultSnackbar, setOpenSubmitResultSnackbar] = React.useState(false);
  const [snackbarMessage, setSnackbarMessage] = React.useState('');
  const [snackbarSeverity, setSnackbarSeverity] = React.useState<AlertProps['severity']>('success');

  return (
    <>
      <Paper style={{ height: '100%' }}>
        <Grid container direction="column" wrap="nowrap" style={{ height: '100%' }}>
          <TableToolbar onCreateTaskClick={() => setOpenCreateTaskForm(true)} />
          <TableContainer style={{ flex: '1 1 0' }}>
            <Table className={classes.table} stickyHeader size="small">
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
                  <React.Fragment key={task.task_id}>
                    <TableRow key={task.task_id} className={classes.infoRow}>
                      <TableCell>{task.task_id}</TableCell>
                      <TableCell>{task.robot_name}</TableCell>
                      <TableCell>{toRelativeDate(task.start_time)}</TableCell>
                      <TableCell>{toRelativeDate(task.end_time)}</TableCell>
                      <TableCell>{taskStateToStr(task.state)}</TableCell>
                    </TableRow>
                    <TableRow>
                      <TableCell className={classes.phasesCell} colSpan={5}>
                        <TaskPhases taskSummary={task}></TaskPhases>
                      </TableCell>
                    </TableRow>
                  </React.Fragment>
                ))}
              </TableBody>
            </Table>
          </TableContainer>
          {paginationOptions && <TablePagination component="div" {...paginationOptions} />}
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
