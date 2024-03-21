import ChevronRightIcon from '@mui/icons-material/ChevronRight';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import {
  Timeline,
  TimelineConnector,
  TimelineContent,
  TimelineDot,
  TimelineDotProps,
  TimelineItem,
  TimelineOppositeContent,
  TimelineProps,
  TimelineSeparator,
  TreeItem,
  TreeView,
} from '@mui/lab';
import { styled } from '@mui/material';
import Typography from '@mui/material/Typography';
import {
  EventState,
  Phase,
  ApiServerModelsRmfApiTaskStateStatus as TaskStatus,
  TaskState,
} from 'api-client';
import React from 'react';

interface TimeLinePropsWithRef extends TimelineProps {
  ref?: React.RefObject<HTMLUListElement>;
}

const classes = {
  paper: 'timeline-paper',
  secondaryTail: 'timeline-secondary-tail',
  timelineRoot: 'timeline-root',
};

const StyledTimeLine = styled((props: TimeLinePropsWithRef) => <Timeline {...props} />)(
  ({ theme }) => ({
    [`& .${classes.paper}`]: {
      padding: theme.spacing(1),
      marginTop: theme.spacing(1),
      width: '200px',
      overflow: 'auto',
      display: 'inline-block',
      maxHeight: '95%',
    },
    [`& .${classes.secondaryTail}`]: {
      backgroundColor: theme.palette.secondary.main,
    },
    [`&.${classes.timelineRoot}`]: {
      padding: '6px 0px',
    },
  }),
);

function NestedEvents(
  eventStates: { [key: string]: EventState } | undefined,
  eventId: number | undefined,
) {
  if (eventStates !== undefined && eventStates !== null && eventId !== undefined) {
    const event = eventStates[eventId];
    if (event !== undefined) {
      return (
        <TreeItem
          nodeId={`event-${event.id}`}
          key={`event-${event.id}`}
          label={event.name || 'undefined'}
        >
          {event.deps
            ? event.deps.map((childId) => {
                return NestedEvents(eventStates, childId);
              })
            : null}
        </TreeItem>
      );
    }
  }

  return null;
}

function colorDot(phase: Phase | undefined): TimelineDotProps['color'] {
  if (phase == null) return 'error';

  if (phase.final_event_id == null || phase.events == null) return 'grey';

  const root_event = phase.events[phase.final_event_id];
  if (root_event == null) return 'error';

  if (root_event.status == null) return 'error';

  switch (root_event.status) {
    case TaskStatus.Uninitialized:
    case TaskStatus.Blocked:
    case TaskStatus.Error:
    case TaskStatus.Failed:
      return 'error';

    case TaskStatus.Queued:
    case TaskStatus.Standby:
      return 'grey';

    case TaskStatus.Underway:
      return 'success';

    case TaskStatus.Skipped:
    case TaskStatus.Canceled:
    case TaskStatus.Killed:
      return 'secondary';

    case TaskStatus.Delayed:
      return 'warning';

    case TaskStatus.Completed:
      return 'primary';

    default:
      return 'error';
  }
}

export interface TaskTimelineProps {
  taskState: TaskState;
}

function RenderPhase(phase: Phase) {
  return (
    <TimelineItem key={phase.id} sx={{ width: 0 }}>
      <TimelineOppositeContent color="text.secondary">
        <Typography variant="overline" color="textSecondary" style={{ textAlign: 'justify' }}>
          {phase.unix_millis_start_time != null
            ? new Date(phase.unix_millis_start_time).toLocaleTimeString()
            : null}
        </Typography>
      </TimelineOppositeContent>
      <TimelineSeparator>
        <TimelineDot color={colorDot(phase)} />
        <TimelineConnector />
      </TimelineSeparator>
      <TimelineContent>
        <Typography variant="overline" color="textSecondary" style={{ textAlign: 'justify' }}>
          {phase.id}. {phase.category}
        </Typography>
        <TreeView defaultCollapseIcon={<ExpandMoreIcon />} defaultExpandIcon={<ChevronRightIcon />}>
          {phase.events && NestedEvents(phase.events, phase.final_event_id)}
        </TreeView>
      </TimelineContent>
    </TimelineItem>
  );
}

export function TaskTimeline({ taskState }: TaskTimelineProps): JSX.Element {
  const phases = taskState.phases ? Object.values(taskState.phases) : [];
  return (
    <StyledTimeLine className={classes.timelineRoot}>
      {phases.map((phase) => RenderPhase(phase))}
    </StyledTimeLine>
  );
}
