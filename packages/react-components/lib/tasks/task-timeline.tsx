import Paper from '@mui/material/Paper';
import Typography from '@mui/material/Typography';
import { styled } from '@mui/material';
import {
  Timeline,
  TimelineConnector,
  TimelineContent,
  TimelineDot,
  TimelineItem,
  TimelineOppositeContent,
  TimelineSeparator,
  TimelineProps,
  TreeView,
  TreeItem,
} from '@mui/lab';
import { TaskState } from 'api-client';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import ChevronRightIcon from '@mui/icons-material/ChevronRight';
import React from 'react';
import { getTreeViewHeader } from './utils';
import { format } from 'date-fns';

interface TimeLinePropsWithRef extends TimelineProps {
  ref?: React.RefObject<HTMLUListElement>;
}

const classes = {
  paper: 'timeline-paper',
  secondaryTail: 'timeline-secondary-tail',
  pendingPhase: 'timeline-pending-phase',
  completedPhase: 'timeline-completed-phase',
  failedPhase: 'timeline-failed-phase',
  timelineRoot: 'timeline-root',
};
const StyledTimeLine = styled((props: TimeLinePropsWithRef) => <Timeline {...props} />)(
  ({ theme }) => ({
    [`& .${classes.paper}`]: {
      padding: theme.spacing(1),
      marginTop: theme.spacing(1),
      width: '200px',
      maxHeight: '100px',
      overflow: 'auto',
      display: 'inline-block',
    },
    [`& .${classes.secondaryTail}`]: {
      backgroundColor: theme.palette.secondary.main,
    },
    [`& .${classes.pendingPhase}`]: {
      background: theme.palette.info.light,
    },
    [`& .${classes.completedPhase}`]: {
      background: theme.palette.success.light,
    },
    [`& .${classes.failedPhase}`]: {
      background: theme.palette.error.light,
    },
    [`&.${classes.timelineRoot}`]: {
      padding: '6px 0px',
    },
  }),
);

interface TaskTreeProps {
  task: TaskState;
}

function LoopTaskTree({ task }: TaskTreeProps) {
  const timelinePhases = task.phases;
  // TODO - remove if a timeline for estimated fisnishing time is not needed
  // let totalTimeTaken = 0;
  // timelinePhases &&
  //   Object.keys(timelinePhases).forEach((p) => {
  //     const estimateMillis = timelinePhases[p].estimate_millis;
  //     if (estimateMillis) totalTimeTaken += estimateMillis * 1000;
  //   });
  function getTimeLineDotProps(taskState: TaskState) {
    if (taskState.active) return { className: classes.completedPhase };
    else {
      return { className: classes.failedPhase };
    }
  }
  return timelinePhases ? (
    <TimelineItem key={'Loop'}>
      <TimelineOppositeContent style={{ flex: 1, padding: '2px 2px' }}>
        <Typography variant="overline" color="textSecondary" style={{ textAlign: 'justify' }}>
          {format(new Date(), "hh:mm aaaaa'm'")}
        </Typography>
      </TimelineOppositeContent>
      <TimelineSeparator>
        <TimelineDot {...getTimeLineDotProps(task)} />
        <TimelineConnector />
      </TimelineSeparator>
      <TimelineContent>
        <TreeView defaultCollapseIcon={<ExpandMoreIcon />} defaultExpandIcon={<ChevronRightIcon />}>
          <TreeItem nodeId="node1" label={getTreeViewHeader(task.category)}>
            <TreeItem
              nodeId="node2"
              label={Object.keys(timelinePhases).map((phase, idx) => {
                return (
                  <Paper className={classes.paper} key={idx}>
                    <Typography variant="caption">{timelinePhases[phase].detail}</Typography>
                  </Paper>
                );
              })}
            />
          </TreeItem>
        </TreeView>
      </TimelineContent>
    </TimelineItem>
  ) : null;
}

function DeliveryTaskTree({ task }: TaskTreeProps) {
  // TODO - get timeline dot props, get ingestor and dispenser information?
  return (
    <TimelineItem key={'Delivery'}>
      <TimelineOppositeContent style={{ flex: 1, padding: '2px 2px' }}>
        <Typography variant="overline" color="textSecondary" style={{ textAlign: 'justify' }}>
          {format(new Date(), "hh:mm aaaaa'm'")}
        </Typography>
      </TimelineOppositeContent>
      <TimelineSeparator>
        <TimelineDot />
        <TimelineConnector />
      </TimelineSeparator>
      <TimelineContent>
        <TreeView defaultCollapseIcon={<ExpandMoreIcon />} defaultExpandIcon={<ChevronRightIcon />}>
          <TreeItem nodeId="node1" label={getTreeViewHeader(task.category)}>
            <TreeItem nodeId="node2" label={task.category} />
          </TreeItem>
        </TreeView>
      </TimelineContent>
    </TimelineItem>
  );
}

export interface TaskTimelineProps {
  taskState: TaskState;
}

export function TaskTimeline({ taskState }: TaskTimelineProps): JSX.Element {
  // TODO - leaving here for reference for other treeviews
  // function getTimeLineDotProps(taskState: TaskState, taskPhase: Phase) {
  //   if (taskState.completed?.includes(taskPhase.id)) return { className: classes.completedPhase };
  //   if (taskPhase.id === taskState.active) return { className: classes.completedPhase };
  //   if (taskState.pending?.includes(taskPhase.id)) return { className: classes.completedPhase };
  //   else {
  //     return { className: classes.failedPhase };
  //   }
  // }
  function GetTreeView(category: string) {
    if (category.includes('Loop')) return <LoopTaskTree task={taskState} />;
    if (category.includes('Delivery')) return <DeliveryTaskTree task={taskState} />;
  }

  return (
    <StyledTimeLine className={classes.timelineRoot}>
      {taskState.category ? GetTreeView(taskState.category) : null}
    </StyledTimeLine>
  );
}
