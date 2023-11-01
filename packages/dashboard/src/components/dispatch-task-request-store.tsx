import {
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  Divider,
  TextField,
  Typography,
} from '@mui/material';
import { TaskState } from 'api-client';
import React from 'react';
import { Subscription } from 'rxjs';
import { AppControllerContext } from './app-contexts';
import { RmfAppContext } from './rmf-app';

interface Task {
  data: TaskState[];
}

export const DispatchTaskStore = () => {
  const [isOpen, setIsOpen] = React.useState(false);
  const { showAlert } = React.useContext(AppControllerContext);
  const rmf = React.useContext(RmfAppContext);
  const [tasksState, setTasksState] = React.useState<Task>({ data: [] });
  const [tasksIds, setTasksIds] = React.useState<string[]>([]);
  const [enableSumit, setSumit] = React.useState(true);
  const [dispatched, setDispatched] = React.useState(false);

  const GET_LIMIT = 10;
  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const subs: Subscription[] = [];
    const timer = setInterval(() => {
      (async () => {
        const resp = await rmf.tasksApi.queryTaskStatesTasksGet(
          undefined,
          undefined,
          undefined,
          'queued',
          undefined,
          undefined,
          GET_LIMIT,
          undefined,
          undefined,
          undefined,
        );
        const results = resp.data as TaskState[];
        const newTasks = results.slice(0, GET_LIMIT);

        setTasksState((old) => ({
          ...old,
          data: newTasks,
        }));

        subs.push(
          ...newTasks.map((task) =>
            rmf
              .getTaskStateObs(task.booking.id)
              .subscribe((task) => setTasksState((prev) => ({ ...prev, [task.booking.id]: task }))),
          ),
        );
      })();
      setIsOpen(true);
      return () => subs.forEach((s) => s.unsubscribe());
    }, 10000);

    return () => {
      clearInterval(timer);
    };
  }, [rmf]);

  const handleCancelTaskClick = React.useCallback<React.MouseEventHandler>(async () => {
    try {
      if (!rmf) {
        throw new Error('tasks api not available');
      }
      for (let index = 0; index < tasksState.data.length; index++) {
        await rmf.tasksApi?.postCancelTaskTasksCancelTaskPost({
          type: 'cancel_task_request',
          task_id: tasksState.data[index].booking.id,
        });
        if (tasksState.data[index].booking.requester) {
          setTasksIds((prevTasksIds) => [...prevTasksIds, tasksState.data[index].booking.id]);
        }
      }

      setDispatched(false);
      setSumit(false);
    } catch (e) {
      console.error(e);
    }
  }, [rmf, tasksState.data]);

  const handleSubmitTask = React.useCallback<React.MouseEventHandler>(async () => {
    try {
      if (!rmf) {
        throw new Error('tasks api not available');
      }
      for (let index = 0; index < tasksIds.length; index++) {
        const request = await (
          await rmf.tasksApi.getTaskRequestTasksTaskIdRequestGet(tasksIds[index])
        ).data.request.request;

        await rmf.tasksApi.postDispatchTaskTasksDispatchTaskPost({
          type: 'dispatch_task_request',
          request,
        });
      }
      showAlert('success', 'Successfully created task');
      setDispatched(true);
      setSumit(true);
      setIsOpen(false);
    } catch (e) {
      console.error(e);
    }
  }, [rmf, tasksIds, showAlert]);

  return (
    <>
      <Dialog
        PaperProps={{
          style: {
            boxShadow: 'none',
          },
        }}
        maxWidth="sm"
        fullWidth={true}
        open={isOpen}
      >
        <DialogTitle align="center">Dispatch tasks</DialogTitle>
        <Divider />
        <DialogContent>
          <Typography variant="h6" gutterBottom>
            Task IDs:
          </Typography>
          <TextField
            multiline
            variant="outlined"
            InputProps={{
              readOnly: true,
            }}
            fullWidth
            value={tasksState.data.map((task) => `• ${task.booking.id}`).join('\n')}
          />
        </DialogContent>
        <DialogActions>
          <Button
            size="small"
            variant="contained"
            onClick={handleCancelTaskClick}
            disabled={!enableSumit}
            autoFocus
          >
            {`${dispatched ? 'Cancelled' : 'Cancel Tasks'} `}
          </Button>

          <Button
            size="small"
            variant="contained"
            onClick={handleSubmitTask}
            disabled={enableSumit}
            autoFocus
          >
            {`${dispatched ? 'Dispatched' : 'Re-Dispatch'} `}
          </Button>

          <Button
            size="small"
            variant="contained"
            onClick={() => {
              setIsOpen(false);
            }}
            disabled={false}
            autoFocus
          >
            Dismiss
          </Button>
        </DialogActions>
      </Dialog>
    </>
  );
};
