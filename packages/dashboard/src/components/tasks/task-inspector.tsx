import { Box, Dialog, DialogContent, DialogTitle, Divider, Grid, Typography } from '@mui/material';
import { CardContent, useTheme } from '@mui/material';
import { TaskEventLog, TaskStateOutput as TaskState } from 'api-client';
import React from 'react';
import { TaskInfo } from 'react-components';

import { useRmfApi } from '../../hooks/use-rmf-api';
import { TaskCancelButton } from './task-cancellation';
import { TaskLogs } from './task-logs';

export interface TableDataGridState {
  task: TaskState | null;
  onClose: () => void;
}

export function TaskInspector({ task, onClose }: TableDataGridState): JSX.Element {
  const theme = useTheme();
  const rmfApi = useRmfApi();

  const [taskState, setTaskState] = React.useState<TaskState | null>(null);
  const [taskLogs, setTaskLogs] = React.useState<TaskEventLog | null>(null);
  const [isOpen, setIsOpen] = React.useState(true);

  React.useEffect(() => {
    if (!task) {
      setTaskState(null);
      setTaskLogs(null);
      return;
    }
    const sub = rmfApi.getTaskStateObs(task.booking.id).subscribe((subscribedTask) => {
      (async () => {
        try {
          const logs = (
            await rmfApi.tasksApi.getTaskLogTasksTaskIdLogGet(
              subscribedTask.booking.id,
              `0,${Number.MAX_SAFE_INTEGER}`,
            )
          ).data;
          setTaskLogs(logs);
        } catch {
          console.log(`Failed to fetch task logs for ${subscribedTask.booking.id}`);
          setTaskLogs(null);
        }
        setTaskState(subscribedTask);
      })();
    });
    return () => sub.unsubscribe();
  }, [rmfApi, task]);

  return (
    <>
      <Grid container wrap="nowrap" direction="column" height="100%">
        <Dialog
          open={isOpen}
          onClose={() => {
            setIsOpen(false);
            onClose();
          }}
          aria-labelledby="alert-dialog-title"
          aria-describedby="alert-dialog-description"
          fullWidth
          maxWidth="md"
        >
          <Grid container spacing={2}>
            <Grid item xs={12}>
              <DialogTitle id="scroll-dialog-title" align="center">
                Task: {task?.booking.id}
              </DialogTitle>
            </Grid>
          </Grid>
          <DialogContent style={{ height: 700 }} dividers={true}>
            <Box component="div" sx={{ position: 'relative' }}>
              <Grid container direction="row" wrap="nowrap" height="100%">
                <Grid item xs={6}>
                  {taskState ? (
                    <>
                      <CardContent sx={{ overflow: 'auto' }}>
                        <TaskInfo task={taskState} title="Details" />
                      </CardContent>
                      <Grid item paddingLeft={2} paddingRight={2}>
                        <TaskCancelButton
                          taskId={taskState ? taskState.booking.id : null}
                          style={{
                            marginTop: theme.spacing(1),
                            marginBottom: theme.spacing(1),
                          }}
                          fullWidth
                          variant="contained"
                          color="secondary"
                          aria-label="Cancel Task"
                        />
                      </Grid>
                    </>
                  ) : (
                    <Grid container wrap="nowrap" alignItems="center" style={{ height: '100%' }}>
                      <CardContent>
                        <Typography variant="h6" align="center">
                          Click on a task to view more information
                        </Typography>
                      </CardContent>
                    </Grid>
                  )}
                </Grid>
                <Divider
                  orientation="vertical"
                  flexItem
                  style={{ marginLeft: theme.spacing(2), marginRight: theme.spacing(2) }}
                />
                <Grid item xs={6}>
                  <CardContent>
                    <TaskLogs taskLog={taskLogs} taskState={taskState} title="Logs" />
                  </CardContent>
                </Grid>
              </Grid>
            </Box>
          </DialogContent>
        </Dialog>
      </Grid>
    </>
  );
}
