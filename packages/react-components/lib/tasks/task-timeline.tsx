import ChevronRightIcon from '@mui/icons-material/ChevronRight';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import {
  Timeline,
  TimelineConnector,
  TimelineContent,
  TimelineDot,
  TimelineItem,
  TimelineOppositeContent,
  TimelineProps,
  TimelineSeparator,
  TreeItem,
  TreeView,
} from '@mui/lab';
import { styled } from '@mui/material';
import Typography from '@mui/material/Typography';
import { TaskState } from 'api-client';
import React from 'react';

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
    [`&.${classes.timelineRoot}`]: {
      padding: '6px 0px',
    },
  }),
);

export interface TaskTimelineProps {
  taskState: TaskState;
}

export function TaskTimeline({ taskState }: TaskTimelineProps): JSX.Element {
  const phases = taskState.phases ? Object.values(taskState.phases) : [];

  return (
    <StyledTimeLine className={classes.timelineRoot}>
      {phases.map((phase, idx) => (
        <TimelineItem key={phase.id}>
          <TimelineOppositeContent style={{ flex: 1, padding: '2px 2px' }}>
            <Typography variant="overline" color="textSecondary" style={{ textAlign: 'justify' }}>
              {phase.unix_millis_start_time
                ? new Date(phase.unix_millis_start_time).toLocaleTimeString()
                : 'unknown'}
            </Typography>
          </TimelineOppositeContent>
          <TimelineSeparator>
            <TimelineDot color="info" />
            {idx < phases.length - 1 && <TimelineConnector />}
          </TimelineSeparator>
          <TimelineContent>
            <TreeView
              defaultCollapseIcon={<ExpandMoreIcon />}
              defaultExpandIcon={<ChevronRightIcon />}
            >
              {/* FIXME: rmf does not return event hierarchy information */}
              {phase.events
                ? Object.values(phase.events).map((event) => (
                    <TreeItem
                      key={event.id}
                      nodeId={event.id.toString()}
                      label={event.name || 'unknown'}
                    ></TreeItem>
                  ))
                : null}
            </TreeView>
          </TimelineContent>
        </TimelineItem>
      ))}
    </StyledTimeLine>
  );
}
