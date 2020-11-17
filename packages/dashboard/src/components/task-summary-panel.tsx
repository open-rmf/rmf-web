import React from 'react';
import Debug from 'debug';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { makeStyles, Typography } from '@material-ui/core';
import {
  SnapshotControlButtonGroup,
  snapshotReducer,
  SnapshotState,
  SnapshotStateType,
  TaskSummaryAccordion,
  SnapshotActionType,
  sortTasksByState,
} from 'react-components';

const debug = Debug('OmniPanel:TaskSummaryPanel');

export interface TaskSummaryAccordionProps {
  tasks: RomiCore.TaskSummary[];
}

export const TaskSummaryPanel = React.memo((props: TaskSummaryAccordionProps) => {
  debug('task summary status panel render');

  const { tasks } = props;
  const classes = useStyles();

  // We need to persist across the renders
  const savedTasksContent = React.useRef<{
    [key: string]: RomiCore.TaskSummary;
  }>({});

  const initialValues: SnapshotState = {
    [SnapshotStateType.Content]: {} as {
      [key: string]: RomiCore.TaskSummary;
    },
  };
  const [stateSnapshot, dispatchSnapshot] = React.useReducer(snapshotReducer, initialValues);

  // TODO: we need to synchronize this with a proper backend when it's ready. Now the completed
  // task will be flushed on a browser refresh because they are being saved in memory.
  const handleClearAllCurrTasks = React.useCallback(() => {
    dispatchSnapshot({ type: SnapshotActionType.Clear, payload: savedTasksContent.current });
  }, []);

  const handleRestoreTasks = React.useCallback(() => {
    dispatchSnapshot({ type: SnapshotActionType.Restore, payload: savedTasksContent.current });
  }, []);

  // Update Task list content
  React.useEffect(() => {
    if (tasks.length === 0) return;
    dispatchSnapshot({ type: SnapshotActionType.AddContent, payload: tasks });
  }, [tasks]);

  // Order by state. Put first the Active -> Queue -> Failed -> Finished
  const currentTaskContents: RomiCore.TaskSummary[] = React.useMemo(() => {
    return sortTasksByState(stateSnapshot[SnapshotStateType.Content]);
  }, [stateSnapshot]);

  return (
    <Typography variant="body1" component={'span'}>
      <div className={classes.buttonGroupDiv}>
        <SnapshotControlButtonGroup
          disableClear={tasks.length === 0}
          disableRestore={tasks.length === 0}
          onRestoreClick={handleRestoreTasks}
          onClearClick={handleClearAllCurrTasks}
          showSave={false}
          showReset={false}
        />
      </div>
      <TaskSummaryAccordion tasks={currentTaskContents} />
    </Typography>
  );
});

const useStyles = makeStyles(() => ({
  buttonGroupDiv: {
    padding: '0.5rem 1rem',
  },
}));

export default TaskSummaryPanel;
