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
import { format } from 'date-fns';
import { TaskState, Phase, EventState } from 'api-client';
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
        <TreeItem nodeId={`event-${event.id}`} label={event.name || 'undefined'}>
          {event.deps
            ? event.deps.map((childId) => {
                return NestedEvents(eventStates, childId);
              })
            : null}
        </TreeItem>
      );
    }
  }
}

function colorDot(
  eventsStates: { [key: string]: EventState } | undefined,
): TimelineDotProps['color'] {
  let standby = true;
  for (const id in eventsStates) {
    const event: EventState = eventsStates[id];
    if (event.status === 'underway') {
      return 'success';
    }
    if (event.status !== 'standby') {
      standby = false;
    }
  }
  if (standby) return 'info';

  return 'error';
}

export interface TaskTimelineProps {
  taskState: TaskState;
}

function RenderPhase(phase: Phase) {
  return (
    <TimelineItem key={phase.id}>
      <TimelineOppositeContent color="text.secondary">
        <Typography variant="overline" color="textSecondary" style={{ textAlign: 'justify' }}>
          {phase.original_estimate_millis
            ? format(new Date(phase.original_estimate_millis), "hh:mm aaaaa'm'")
            : null}
        </Typography>
      </TimelineOppositeContent>
      <TimelineSeparator>
        <TimelineDot color={colorDot(phase.events)} />
        <TimelineConnector />
      </TimelineSeparator>
      <TimelineContent>
        <Typography variant="overline" color="textSecondary" style={{ textAlign: 'justify' }}>
          {phase.id}. {phase.category}
        </Typography>
        <TreeView defaultCollapseIcon={<ExpandMoreIcon />} defaultExpandIcon={<ChevronRightIcon />}>
          {NestedEvents(phase.events, phase.final_event_id)}
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
