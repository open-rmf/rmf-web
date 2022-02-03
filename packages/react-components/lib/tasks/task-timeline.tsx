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
import { format } from 'date-fns';
import { TaskState, EventState } from 'api-client';
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

function nestedEvents(eventsStates: { [key: string]: EventState } | undefined, child: number) {
  if (eventsStates !== undefined) {
    const deps = eventsStates[child] ? eventsStates[child].deps : [];
    return (
      <TreeItem
        key={`event-${child}`}
        nodeId={`parent-${eventsStates[child].name}`}
        label={eventsStates[child].name}
      >
        {deps?.map((id) => {
          if (eventsStates[id].deps?.length) {
            nestedEvents(eventsStates, id);
          } else {
            return (
              <TreeItem
                key={eventsStates[child].name}
                nodeId={`child-${id}`}
                label={eventsStates[id].name}
              ></TreeItem>
            );
          }
        })}
      </TreeItem>
    );
  }
}

function colorDot(eventsStates: { [key: string]: EventState } | undefined) {
  let standby: Boolean = true;
  for (const id in eventsStates) {
    const event: EventState = eventsStates[id];
    if (event.status === 'underway') {
      return 'success' as const;
    }
    if (event.status !== 'standby') {
      standby = false;
    }
  }
  if (standby) return 'info' as const;

  return 'error' as const;
}

export interface TaskTimelineProps {
  taskState: TaskState;
}

export function TaskTimeline({ taskState }: TaskTimelineProps): JSX.Element {
  const phases = taskState.phases ? Object.values(taskState.phases) : [];
  return (
    <StyledTimeLine className={classes.timelineRoot}>
      {phases.map((phase, idx) =>
        phase.events ? (
          <TimelineItem key={phase.id}>
            <TimelineOppositeContent color="text.secondary">
              <Typography variant="overline" color="textSecondary" style={{ textAlign: 'justify' }}>
                {phase.original_estimate_millis
                  ? format(new Date(phase.original_estimate_millis * 1000), "hh:mm aaaaa'm'")
                  : null}
              </Typography>
            </TimelineOppositeContent>
            <TimelineSeparator>
              <TimelineDot
                color={colorDot(phase.events)}
                sx={{
                  margin: '0.5px',
                }}
              />
              {idx < phases.length - 1 && <TimelineConnector />}
            </TimelineSeparator>
            <TimelineContent>
              <TreeView
                defaultCollapseIcon={<ExpandMoreIcon />}
                defaultExpandIcon={<ChevronRightIcon />}
              >
                {Object.values(phase.events).map((event) => (
                  <TreeItem
                    key={event.id}
                    nodeId={event.id.toString()}
                    label={event.name || 'unknown'}
                  >
                    {event.deps && event.deps.length > 0
                      ? Object.values(event.deps).map((childId) =>
                          nestedEvents(phase.events, childId),
                        )
                      : null}
                  </TreeItem>
                ))}
              </TreeView>
            </TimelineContent>
          </TimelineItem>
        ) : null,
      )}
    </StyledTimeLine>
  );
}
