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
import React from 'react';
import * as RmfModels from 'rmf-models';
import { taskStateToStr } from './utils';
import { TaskPhases } from './task-phases';
import AddOutlinedIcon from '@material-ui/icons/AddCircle';

const useStyles = makeStyles((theme) => ({
  table: {
    minWidth: 650,
  },
  title: {
    flex: '1 1 100%',
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
                <TableRow key={task.task_id}>
                  <TableCell>{task.task_id}</TableCell>
                  <TableCell>{task.robot_name}</TableCell>
                  <TableCell>{task.start_time.sec}</TableCell>
                  <TableCell>{task.end_time.sec}</TableCell>
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
