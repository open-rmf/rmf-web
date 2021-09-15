import React from 'react';
import Debug from 'debug';
import * as RmfModels from 'rmf-models';
import { IconButton, Typography, styled } from '@material-ui/core';
import { makeStyles } from '@material-ui/styles';
import {
  MultiSelectTreeViewProps,
  SingleSelectTreeViewProps,
  TreeItem,
  TreeView,
  TreeViewProps,
} from '@material-ui/lab';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import ChevronRightIcon from '@material-ui/icons/ChevronRight';
import PlayCircleFilledWhiteIcon from '@material-ui/icons/PlayCircleFilledWhite';
import PauseCircleFilledIcon from '@material-ui/icons/PauseCircleFilled';
import { Theme } from '@material-ui/core/styles/createTheme';
import { SimpleInfo, SimpleInfoData } from '../simple-info';
import { formatStatus, getActorFromStatus, getStateLabel } from './task-summary-utils';

const debug = Debug('Tasks:TaskSummaryAccordion');

interface TaskSummaryAccordionInfoProps {
  task: RmfModels.TaskSummary;
}
interface TreeViewRootProps extends SingleSelectTreeViewProps {
  onNodeToggle?: MultiSelectTreeViewProps['onNodeSelect'];
}

const classes = {
  root: 'task-summary-root',
  accordionDetailLine: 'accordion-detail-line',
  treeChildren: 'tree-children',
  labelContent: 'label-content',
  expanded: 'expanded',
  completed: 'completed',
  queued: 'queued',
  active: 'active',
  failed: 'failed',
  taskActor: 'task-actor',
  overrideArrayItemValue: 'override-array-item-value',
  overrideContainer: 'override-container',
  overrideValue: 'override-value',
};
const TaskSummaryAccordianRoot = styled((props: TreeViewRootProps) => <TreeView {...props} />)(
  ({ theme }) => ({
    [`& .${classes.root}`]: {
      padding: '1rem',
    },
    [`& .${classes.accordionDetailLine}`]: {
      display: 'flex',
      justifyContent: 'space-between',
      padding: theme.spacing(0.5),
    },
    [`& .${classes.treeChildren}`]: {
      margin: '0.5rem 0',
    },
    [`& .${classes.labelContent}`]: {
      padding: '0.5rem',
      borderRadius: '0.5rem',
      boxShadow: '0 0 25px 0 rgb(72, 94, 116, 0.3)',
      overflowX: 'hidden',
      display: 'flex',
      flexDirection: 'column',
    },
    [`& .${classes.expanded}`]: {
      borderLeft: `0.1rem solid #cccccc`,
    },
    [`& .${classes.completed}`]: {
      backgroundColor: '#4E5453 !important',
    },
    [`& .${classes.queued}`]: {
      backgroundColor: theme.palette.warning.main + '!important',
    },
    [`& .${classes.active}`]: {
      backgroundColor: theme.palette.success.light + '!important',
    },
    [`& .${classes.failed}`]: {
      backgroundColor: theme.palette.error.main + '!important',
    },
    [`& .${classes.taskActor}`]: {
      alignSelf: 'center',
    },
    [`& .${classes.overrideArrayItemValue}`]: {
      textAlign: 'center',
    },
    [`& .${classes.overrideContainer}`]: {
      borderCollapse: 'collapse',
      width: '100%',
      overflowX: 'auto',
    },
    [`& .${classes.overrideValue}`]: {
      display: 'table-cell',
      textAlign: 'end',
      borderBottom: '1px solid',
      borderBottomColor: theme.palette.divider,
      borderTop: '1px solid',
      borderTopColor: theme.palette.divider,
    },
  }),
);

export const TaskSummaryAccordionInfo = (props: TaskSummaryAccordionInfoProps): JSX.Element => {
  const { task } = props;
  const statusDetails = formatStatus(task.status);
  const stateLabel = getStateLabel(task.state);
  const data = [
    { name: 'TaskId', value: task.task_id, wrap: true },
    { name: 'State', value: stateLabel },
    {
      name: 'Status',
      value: statusDetails,
      className: {
        overrideValue: classes.overrideValue,
        overrideArrayItemValue: classes.overrideArrayItemValue,
      },
    },
    { name: 'Subm. Time', value: task.submission_time.sec },
    { name: 'Start time', value: task.start_time.sec },
    { name: 'End Time', value: task.end_time.sec },
  ] as SimpleInfoData[];

  return <SimpleInfo infoData={data} overrideStyle={{ container: classes.overrideContainer }} />;
};

export interface TaskSummaryAccordionProps {
  tasks: RmfModels.TaskSummary[];
}

export const TaskSummaryAccordion = React.memo((props: TaskSummaryAccordionProps) => {
  debug('task summary status panel render');
  const { tasks } = props;
  const [expanded, setExpanded] = React.useState<string[]>([]);
  const [selected, setSelected] = React.useState<string>('');

  const handleToggle: MultiSelectTreeViewProps['onNodeSelect'] = (event, nodeIds) => {
    setExpanded(nodeIds);
  };

  const handleSelect: SingleSelectTreeViewProps['onNodeSelect'] = (event, nodeIds) => {
    setSelected(nodeIds);
  };

  const determineStyle = (state: number): string => {
    switch (state) {
      case RmfModels.TaskSummary.STATE_QUEUED:
        return classes.queued;
      case RmfModels.TaskSummary.STATE_ACTIVE:
        return classes.active;
      case RmfModels.TaskSummary.STATE_COMPLETED:
        return classes.completed;
      case RmfModels.TaskSummary.STATE_FAILED:
        return classes.failed;
      default:
        return 'UNKNOWN';
    }
  };

  const renderActor = (taskStatus: string): React.ReactElement | null => {
    const actor = getActorFromStatus(taskStatus);
    if (!actor) return null;
    return (
      <Typography variant="body1" id="task-actor" className={classes.taskActor}>
        {actor}
      </Typography>
    );
  };

  const renderTaskTreeItem = (task: RmfModels.TaskSummary) => {
    return (
      <TreeItem
        role={'treeitem'}
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
              {task.state === RmfModels.TaskSummary.STATE_ACTIVE && (
                // TODO: add onClick with e.preventDefault() and with the pause plans logic.
                <IconButton disabled>
                  <PlayCircleFilledWhiteIcon />
                </IconButton>
              )}
              {task.state === RmfModels.TaskSummary.STATE_QUEUED && (
                <IconButton disabled>
                  <PauseCircleFilledIcon />
                </IconButton>
              )}
              {task.task_id}
            </Typography>
            {renderActor(task.status)}
          </>
        }
      >
        <TaskSummaryAccordionInfo task={task} key={task.task_id} />
      </TreeItem>
    );
  };

  return (
    <TaskSummaryAccordianRoot
      className={classes.root}
      onNodeSelect={handleSelect}
      onNodeToggle={handleToggle}
      defaultCollapseIcon={<ExpandMoreIcon />}
      defaultExpanded={['root']}
      defaultExpandIcon={<ChevronRightIcon />}
      expanded={expanded}
      selected={selected}
    >
      {tasks.map((task) => renderTaskTreeItem(task))}
    </TaskSummaryAccordianRoot>
  );
});

export default TaskSummaryAccordion;
