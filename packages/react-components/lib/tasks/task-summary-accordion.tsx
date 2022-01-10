import React from 'react';
import Debug from 'debug';
// import type { TaskSummary } from 'api-client';
import { TaskSummary as RmfTaskSummary } from 'rmf-models';
import { IconButton, Typography, styled } from '@mui/material';
import { MultiSelectTreeViewProps, SingleSelectTreeViewProps, TreeItem, TreeView } from '@mui/lab';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import ChevronRightIcon from '@mui/icons-material/ChevronRight';
import PlayCircleFilledWhiteIcon from '@mui/icons-material/PlayCircleFilledWhite';
import PauseCircleFilledIcon from '@mui/icons-material/PauseCircleFilled';
import { SimpleInfo, SimpleInfoData } from '../simple-info';
import { formatStatus, getActorFromStatus, getStateLabel } from './task-summary-utils';

const debug = Debug('Tasks:TaskSummaryAccordion');

interface TaskSummaryAccordionInfoProps {
  task: any;
}
interface TreeViewRootProps extends SingleSelectTreeViewProps {
  onNodeToggle?: MultiSelectTreeViewProps['onNodeSelect'];
}

const classes = {
  root: 'task-summary-accordion-root',
  accordionDetailLine: 'task-summary-accordion-accordion-detail-line',
  treeChildren: 'task-summary-accordion-tree-children',
  labelContent: 'task-summary-accordion-label-content',
  expanded: 'task-summary-accordion-expanded',
  completed: 'task-summary-accordion-completed',
  queued: 'task-summary-accordion-queued',
  active: 'task-summary-accordion-active',
  failed: 'task-summary-accordion-failed',
  taskActor: 'task-summary-accordion-task-actor',
  overrideArrayItemValue: 'task-summary-accordion-override-array-item-value',
  overrideContainer: 'task-summary-accordion-override-container',
  overrideValue: 'task-summary-accordion-override-value',
};
const StyledTreeView = styled((props: TreeViewRootProps) => <TreeView {...props} />)(
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
  tasks: any[];
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
      case RmfTaskSummary.STATE_QUEUED:
        return classes.queued;
      case RmfTaskSummary.STATE_ACTIVE:
        return classes.active;
      case RmfTaskSummary.STATE_COMPLETED:
        return classes.completed;
      case RmfTaskSummary.STATE_FAILED:
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

  const renderTaskTreeItem = (task: any) => {
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
              {task.state === RmfTaskSummary.STATE_ACTIVE && (
                // TODO: add onClick with e.preventDefault() and with the pause plans logic.
                <IconButton disabled>
                  <PlayCircleFilledWhiteIcon />
                </IconButton>
              )}
              {task.state === RmfTaskSummary.STATE_QUEUED && (
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
    <StyledTreeView
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
    </StyledTreeView>
  );
});

export default TaskSummaryAccordion;
