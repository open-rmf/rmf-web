import { TaskState } from 'api-client';
import React from 'react';
import { Subscription } from 'rxjs';
import { RmfAppContext } from './rmf-app';
import { useTaskStore } from './store';
import { AlertComponentStore } from './task-alert';

export const AlertComponent = React.memo(() => {
  const rmf = React.useContext(RmfAppContext);

  const setTaskStore = useTaskStore((state) => state.setTask);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const subs: Subscription[] = [];
    (async () => {
      const resp = await rmf.tasksApi.queryTaskStatesTasksGet(
        undefined,
        undefined,
        undefined,
        'underway',
        undefined,
        undefined,
        undefined,
        undefined,
        undefined,
      );

      const newTasks = resp.data as TaskState[];

      subs.push(
        ...newTasks.map((task) =>
          rmf.getTaskStateObs(task.booking.id).subscribe((task) => {
            if (task.status === 'underway') {
              setTaskStore([...newTasks, task]);
            }
          }),
        ),
      );
      return () => subs.forEach((s) => s.unsubscribe());
    })();
  }, [rmf, setTaskStore]);

  return (
    <div>
      <div>
        <AlertComponentStore />
      </div>
    </div>
  );
});
