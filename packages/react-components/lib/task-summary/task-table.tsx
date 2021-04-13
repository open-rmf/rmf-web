import {
  Paper,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
} from '@material-ui/core';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { taskStateToStr } from './utils';

export interface TaskTableProps {
  tasks: RmfModels.TaskSummary[];
}

export function TaskTable({ tasks }: TaskTableProps): JSX.Element {
  return (
    <TableContainer component={Paper}>
      <Table stickyHeader={true}>
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
            <TableRow key={task.task_id}>
              <TableCell>{task.task_id}</TableCell>
              <TableCell>{task.robot_name}</TableCell>
              <TableCell>{task.start_time.sec}</TableCell>
              <TableCell>{task.end_time.sec}</TableCell>
              <TableCell>{taskStateToStr(task.state)}</TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </TableContainer>
  );
}
