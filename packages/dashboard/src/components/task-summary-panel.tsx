import React from 'react';
import Debug from 'debug';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { IconButton, makeStyles, Typography } from '@material-ui/core';
import { TreeItem, TreeView } from '@material-ui/lab';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import ChevronRightIcon from '@material-ui/icons/ChevronRight';
import { TaskSummaryPanelItem } from './task-summary-panel-item';
import PlayCircleFilledWhiteIcon from '@material-ui/icons/PlayCircleFilledWhite';
import PauseCircleFilledIcon from '@material-ui/icons/PauseCircleFilled';
import { colorPalette } from '../util/css-utils';
import { TreeButtonGroup } from './tree-button-group';
import { Theme } from '@material-ui/core/styles/createMuiTheme';

const debug = Debug('OmniPanel:TaskSummaryPanel');

// TODO: this is a hacky solution to get the actor from the task status,
// this should be changed when then backend sends an actor on the
// taskSummary message (we should use the actor provided by the message).
// https://github.com/osrf/rmf_core/issues/205
export const getActorFromStatus = (status: string) => {
  // Gets the name of the robot if it has any
  // eslint-disable-next-line
  return status.match(/\[[A-Za-z]([a-zA-Z0-9\/]){3,}\]+/gi);
};

export interface TaskSummaryPanelProps {
  allTasks: RomiCore.TaskSummary[];
}

export const TaskSummaryPanel = React.memo((props: TaskSummaryPanelProps) => {
  debug('task summary status panel render');

  const { allTasks } = props;
  const classes = useStyles();

  const [taskContents, setTaskContents] = React.useState<{
    [key: string]: JSX.Element;
  }>({});
  const [expanded, setExpanded] = React.useState<string[]>([]);
  const [selected, setSelected] = React.useState<string>('');

  // We need to persist across the renders
  const savedTasksContent = React.useRef<{
    [key: string]: JSX.Element;
  }>({});

  const handleToggle = (event: React.ChangeEvent<{}>, nodeIds: string[]) => {
    setExpanded(nodeIds);
  };

  const handleSelect = (event: React.ChangeEvent<{}>, nodeIds: string) => {
    setSelected(nodeIds);
  };

  const handleResetTasks = () => {
    setExpanded([]);
    setSelected('');
  };

  const handleClearAllCurrTasks = () => {
    savedTasksContent.current = taskContents;
    setTaskContents({});
  };

  const handleRestoreTasks = () => {
    setTaskContents((currentContent) => {
      // Assigning another reference.
      let taskContentsTemp = Object.assign({}, currentContent);
      // We cannot assign directly the stored values to `currentContent` because it is updating
      // the taskContents too, so when we set the taskContent new value with `setTaskContents`
      // it'll no re-render because it'll not detect any changes.
      Object.keys(savedTasksContent.current).forEach((element) => {
        if (!(element in currentContent)) {
          taskContentsTemp[element] = savedTasksContent.current[element];
        }
      });
      return taskContentsTemp;
    });
    savedTasksContent.current = {};
  };

  const determineStyle = (state: number): string => {
    switch (state) {
      case RomiCore.TaskSummary.STATE_QUEUED:
        return classes.queued;
      case RomiCore.TaskSummary.STATE_ACTIVE:
        return classes.active;
      case RomiCore.TaskSummary.STATE_COMPLETED:
        return classes.completed;
      case RomiCore.TaskSummary.STATE_FAILED:
        return classes.failed;
      default:
        return 'UNKNOWN';
    }
  };

  // Update Task list content
  React.useEffect(() => {
    const renderActor = (taskStatus: string): React.ReactElement | undefined => {
      const actor = getActorFromStatus(taskStatus);
      if (!actor) return;
      return (
        <Typography variant="body1" className={classes.taskActor}>
          {actor}
        </Typography>
      );
    };

    const renderTaskTreeItem = (task: RomiCore.TaskSummary) => {
      return (
        <TreeItem
          data-component="TreeItem"
          nodeId={task.task_id}
          key={task.task_id}
          classes={{
            label: `${determineStyle(task.state)} ${classes.labelContent}`,
            root: classes.treeChildren,
            expanded: classes.expanded,
          }}
          label={
            <>
              <Typography variant="body1" noWrap>
                {task.state === RomiCore.TaskSummary.STATE_ACTIVE && (
                  <IconButton
                    // TODO: replace onClick with the pause plans logic.
                    onClick={(e) => {
                      e.preventDefault();
                      console.log('Not implemented');
                    }}
                  >
                    <PlayCircleFilledWhiteIcon />
                  </IconButton>
                )}
                {task.state === RomiCore.TaskSummary.STATE_QUEUED && (
                  <IconButton
                    // TODO: replace onClick with the pause plans logic.
                    onClick={(e) => {
                      e.preventDefault();
                      console.log('Not implemented');
                    }}
                  >
                    <PauseCircleFilledIcon />
                  </IconButton>
                )}
                {task.task_id}
              </Typography>
              {renderActor(task.status)}
            </>
          }
        >
          <TaskSummaryPanelItem task={task} key={task.task_id} />
        </TreeItem>
      );
    };

    if (allTasks.length === 0) return;

    setTaskContents((currentContent) => {
      // Assigning another reference.
      let taskContentsTemp = Object.assign({}, currentContent);
      // We cannot assign directly the stored values to `currentContent` because it is updating
      // the taskContents too, so when we set the taskContent new value with `setTaskContents`
      // it'll no re-render because it'll not detect any changes.
      allTasks.forEach((task) => {
        taskContentsTemp[task.task_id] = renderTaskTreeItem(task);
      });
      return taskContentsTemp;
    });

    // Ignoring dependency taskContents, it'll enter in a infinite loop
    // eslint-disable-next-line
  }, [allTasks]);

  return (
    <Typography variant="body1" component={'span'}>
      <div className={classes.buttonGroupDiv}>
        <TreeButtonGroup
          disableReset={allTasks.length === 0}
          disableClear={allTasks.length === 0}
          disableRestore={allTasks.length === 0}
          handleResetClick={handleResetTasks}
          handleRestoreClick={handleRestoreTasks}
          handleClearClick={handleClearAllCurrTasks}
        />
      </div>
      <TreeView
        className={classes.root}
        onNodeSelect={handleSelect}
        onNodeToggle={handleToggle}
        defaultCollapseIcon={<ExpandMoreIcon />}
        defaultExpanded={['root']}
        defaultExpandIcon={<ChevronRightIcon />}
        expanded={expanded}
        selected={selected}
      >
        {Object.keys(taskContents)
          .reverse()
          .map((key) => taskContents[key])}
      </TreeView>
    </Typography>
  );
});

const useStyles = makeStyles((theme: Theme) => ({
  root: {
    padding: '1rem',
  },
  accordionDetailLine: {
    display: 'flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },
  treeChildren: {
    margin: '0.5rem 0',
  },
  labelContent: {
    padding: '0.5rem',
    borderRadius: '0.5rem',
    boxShadow: '0 0 25px 0 rgb(72, 94, 116, 0.3)',
    overflowX: 'hidden',
    display: 'flex',
    flexDirection: 'column',
  },
  expanded: {
    borderLeft: `0.1rem solid ${colorPalette.unknown}`,
  },
  completed: {
    backgroundColor: '#4E5453',
  },
  queued: {
    backgroundColor: theme.palette.warning.main,
  },
  active: {
    backgroundColor: theme.palette.success.light,
  },
  failed: {
    backgroundColor: theme.palette.error.main,
  },
  taskActor: {
    alignSelf: 'center',
  },
  buttonGroupDiv: {
    padding: '0.5rem 1rem',
  },
}));

export default TaskSummaryPanel;
