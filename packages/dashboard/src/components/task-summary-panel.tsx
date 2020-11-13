import React from 'react';
import Debug from 'debug';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { makeStyles, Typography } from '@material-ui/core';
import { SnapshotControlButtonGroup, TaskSummaryAccordion } from 'react-components';

const debug = Debug('OmniPanel:TaskSummaryAccordion');

export interface TaskSummaryAccordionProps {
  tasks: RomiCore.TaskSummary[];
}

export const TaskSummaryPanel = React.memo((props: TaskSummaryAccordionProps) => {
  debug('task summary status panel render');

  const { tasks } = props;
  const classes = useStyles();

  const [taskContents, setTaskContents] = React.useState<{
    [key: string]: RomiCore.TaskSummary;
  }>({});

  // We need to persist across the renders
  const savedTasksContent = React.useRef<{
    [key: string]: RomiCore.TaskSummary;
  }>({});

  // TODO: we need to synchronize this with a proper backend when it's ready. Now the completed
  // task will be flushed on a browser refresh because they are being saved in memory.
  const handleClearAllCurrTasks = () => {
    savedTasksContent.current = Object.assign({}, taskContents);
    setTaskContents({});
  };

  const handleRestoreTasks = () => {
    // Adding this because the test interpreter cannot find the reference to `savedTasksContent current`
    // inside the setTaskContents
    const storedTasks = savedTasksContent.current;
    setTaskContents((currentContent) => {
      // Assigning another reference.
      const newTaskContents = Object.assign({}, currentContent);
      // We cannot assign directly the stored values to `currentContent` because it is updating
      // the taskContents too, so when we set the taskContent new value with `setTaskContents`
      // it'll no re-render because it'll not detect any changes.
      Object.keys(storedTasks).forEach((element) => {
        if (!(element in currentContent)) {
          newTaskContents[element] = storedTasks[element];
        }
      });
      return newTaskContents;
    });
    savedTasksContent.current = {};
  };

  // Update Task list content
  React.useEffect(() => {
    if (tasks.length === 0) return;

    setTaskContents((currentContent) => {
      // Assigning another reference.
      const newTaskContents = Object.assign({}, currentContent);
      // We cannot assign directly the stored values to `currentContent` because it is updating
      // the taskContents too, so when we set the taskContent new value with `setTaskContents`
      // it'll no re-render because it'll not detect any changes.
      tasks.forEach((task) => {
        newTaskContents[task.task_id] = task;
      });
      return newTaskContents;
    });
  }, [tasks, classes]);

  const taskContents2: RomiCore.TaskSummary[] = React.useMemo(() => {
    return Object.values(taskContents);
  }, [taskContents]);

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
      <TaskSummaryAccordion tasks={taskContents2} />
    </Typography>
  );
});

const useStyles = makeStyles(() => ({
  buttonGroupDiv: {
    padding: '0.5rem 1rem',
  },
}));

export default TaskSummaryPanel;
