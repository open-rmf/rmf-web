import React from 'react';
import { SpotlightValue } from './spotlight-value';
import Debug from 'debug';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { IconButton, makeStyles, Typography } from '@material-ui/core';
import { TreeItem, TreeView } from '@material-ui/lab';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import ChevronRightIcon from '@material-ui/icons/ChevronRight';
import { TaskSummaryPanelItem } from './task-summary-panel-item';
import PlayCircleFilledWhiteIcon from '@material-ui/icons/PlayCircleFilledWhite';
import TaskManager from '../managers/task-manager';
import { colorPalette } from '../util/css-utils';
import { TreeButtonGroup } from './tree-button-group';

const debug = Debug('OmniPanel:TaskSummaryPanel');

export interface TaskSummaryPanelProps {
  allTasks: RomiCore.TaskSummary[];
  spotlight?: Readonly<SpotlightValue<string>>;
}

export const TaskSummaryPanel = React.memo((props: TaskSummaryPanelProps) => {
  debug('task summary status panel render');

  const { allTasks, spotlight } = props;
  const classes = useStyles();

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    // TODO: spotlight
  }, [spotlight]);

  const [taskContents, setTaskContents] = React.useState<{
    [key: string]: JSX.Element;
  }>({});
  const [expanded, setExpanded] = React.useState<string[]>([]);
  const [selected, setSelected] = React.useState<string>('');
  const [savedTasksContent, setSavedTasksContent] = React.useState<{
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
    setSavedTasksContent(taskContents);
    setTaskContents({});
  };

  const handleRestoreTasks = () => {
    // Assigning another reference
    let taskContentsTemp = Object.assign({}, taskContents);
    Object.keys(savedTasksContent).forEach((element) => {
      console.log(!(element in taskContents));
      if (!(element in taskContents)) {
        taskContentsTemp[element] = savedTasksContent[element];
      }
    });
    setTaskContents(taskContentsTemp);
    setSavedTasksContent({}); // setTaskContents({});
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

  const renderActor = (taskStatus: string): React.ReactElement | undefined => {
    const actor = TaskManager.getActorFromStatus(taskStatus);
    if (!actor) return;
    return (
      <Typography variant="body1" className={classes.taskActor}>
        {actor}
      </Typography>
    );
  };

  // Set it like useCallback because is used inside a hook
  const renderTaskTreeItem = React.useCallback(
    (task: RomiCore.TaskSummary) => {
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
                <IconButton
                  // TODO: replace onClick with the pause plans logic.
                  onClick={(e) => {
                    e.preventDefault();
                    console.log('Not implemented');
                  }}
                >
                  <PlayCircleFilledWhiteIcon />
                </IconButton>
                {task.task_id}
              </Typography>
              {renderActor(task.status)}
            </>
          }
        >
          <TaskSummaryPanelItem task={task} key={task.task_id} />
        </TreeItem>
      );
    },
    [classes],
  );

  // Update Task list content
  React.useEffect(() => {
    if (allTasks.length === 0) return;
    let taskContentsTemp = Object.assign({}, taskContents);
    allTasks.forEach((task) => {
      taskContentsTemp[task.task_id] = renderTaskTreeItem(task);
    });
    setTaskContents(taskContentsTemp);
  }, [allTasks]);

  return (
    <Typography variant="body1" component={'span'}>
      <div className={classes.buttonGroupDiv}>
        <TreeButtonGroup
          disableReset={allTasks.length === 0}
          disableClear={allTasks.length === 0}
          disableRestore={allTasks.length === 0}
          handlResetClick={handleResetTasks}
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

const useStyles = makeStyles((theme) => ({
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
