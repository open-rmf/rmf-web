import React from 'react';
import Debug from 'debug';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { makeStyles, Typography } from '@material-ui/core';
import { SnapshotControlButtonGroup, TaskSummaryAccordion } from 'react-components';
import { sortTasksByState } from 'react-components/lib/task-summary/task-summary-utils';

const debug = Debug('OmniPanel:TaskSummaryAccordion');

// TODO: Redux state with actions
function mergeContent(
  currentContent: any,
  storedContent: any,
  replaceCurrentContent: boolean = true,
) {
  const newContents = Object.assign({}, currentContent);
  Object.keys(storedContent).forEach((element) => {
    if (replaceCurrentContent) newContents[element] = storedContent[element];
    else {
      // If the element is already on the currentContent do not replace it
      if (!(element in currentContent)) newContents[element] = storedContent[element];
    }
  });
  return newContents;
}

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
  const handleClearAllCurrTasks = React.useCallback(() => {
    savedTasksContent.current = Object.assign({}, taskContents);
    setTaskContents({});
  }, [taskContents]);

  const handleRestoreTasks = React.useCallback(() => {
    setTaskContents((currentContent) =>
      mergeContent(currentContent, savedTasksContent.current, false),
    );
    savedTasksContent.current = {};
  }, []);

  // Update Task list content
  React.useEffect(() => {
    if (tasks.length === 0) return;
    setTaskContents((currentContent) => mergeContent(currentContent, tasks));
  }, [tasks]);

  //TODO: Order by state. Put first the Active -> Queue -> Failed -> Finished
  const currentTaskContents: RomiCore.TaskSummary[] = React.useMemo(() => {
    return sortTasksByState(taskContents);
  }, [taskContents]);
  console.log(JSON.stringify(currentTaskContents));

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
