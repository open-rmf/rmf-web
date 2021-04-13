import {
  IconButton,
  makeStyles,
  Paper,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Toolbar,
  Typography,
} from '@material-ui/core';
import AddOutlinedIcon from '@material-ui/icons/AddCircle';
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
  infoRow: {
    '& > *': {
      borderBottom: 'unset',
    },
  },
  phasesCell: {
    padding: `0 ${theme.spacing(1)}px`,
  },
}));

function TableToolbar() {
  const classes = useStyles();
  return (
    <Toolbar>
      <Typography className={classes.title} variant="h6">
        Tasks
      </Typography>
      <IconButton>
        <AddOutlinedIcon />
      </IconButton>
    </Toolbar>
  );
}

const toRelativeDate = (rosTime: RmfModels.Time) => {
  return formatDistanceToNow(rosTimeToJs(rosTime), { addSuffix: true });
};

export interface TaskTableProps {
  tasks: RmfModels.TaskSummary[];
}

export function TaskTable({ tasks }: TaskTableProps): JSX.Element {
  const classes = useStyles();

  return (
    <Paper>
      <TableToolbar />
      <TableContainer>
        <Table className={classes.table} stickyHeader={true} size="small">
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
    </Paper>
  );
}
