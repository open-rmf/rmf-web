import { Paper, styled, Typography, useTheme, PaperProps } from '@mui/material';
import { TaskEventLog, TaskState, EventState } from 'api-client';
import Timeline from '@mui/lab/Timeline';
import TimelineItem from '@mui/lab/TimelineItem';
import TimelineSeparator from '@mui/lab/TimelineSeparator';
import TimelineConnector from '@mui/lab/TimelineConnector';
import TimelineContent from '@mui/lab/TimelineContent';
import TimelineDot from '@mui/lab/TimelineDot';
import TimelineOppositeContent from '@mui/lab/TimelineOppositeContent';

import TreeView from '@mui/lab/TreeView';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import ChevronRightIcon from '@mui/icons-material/ChevronRight';
import TreeItem from '@mui/lab/TreeItem';
import { format } from 'date-fns';
import { TimelineDotProps } from '@mui/lab';
import React from 'react';

const prefix = 'task-progress';
const classes = {
  root: `${prefix}-root`,
};

interface TaskLogProps {
  taskLog: TaskEventLog;
  taskState: TaskState;
  fetchTaskLogs?: () => Promise<never[] | undefined>;
}

const Item = styled((props: PaperProps) => <Paper {...props} />)(({ theme }) => ({
  ...theme.typography.body2,
  color: theme.palette.text.secondary,
  width: 290,
}));

function nestedEvents(eventsStates: EventState[], child: number) {
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
              key={`event--${child}`}
              nodeId={`child-${id}`}
              label={eventsStates[id].name}
            ></TreeItem>
          );
        }
      })}
    </TreeItem>
  );
}

function colorDot(eventsStates: EventState[]) {
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

export function TaskProgress(props: TaskLogProps) {
  const { taskLog } = props;
  const { taskState } = props;
  const theme = useTheme();
  const phaseIds = taskLog.phases ? Object.keys(taskLog.phases) : [];

  return (
    <Paper
      sx={{ padding: theme.spacing(1) }}
      variant="outlined"
      key={`task - ${taskState.booking.id}`}
    >
      <Timeline className={classes.root} position="right" key={`task - ${taskState.booking.id}`}>
        {phaseIds.length > 0 ? (
          phaseIds.map((id: string) => {
            const phaseStateObj: any = taskState.phases ? taskState.phases[id] : null;
            const eventsStates = phaseStateObj ? phaseStateObj.events : {};
            const eventIds = eventsStates ? Object.keys(eventsStates) : [];
            return (
              <React.Fragment key={`task - ${taskState.booking.id}`}>
                {eventIds.length > 0 ? (
                  <TimelineItem key={`timeline-item-${id}`}>
                    <TimelineOppositeContent color="text.secondary">
                      {format(
                        new Date(phaseStateObj.original_estimate_millis * 1000),
                        "hh:mm aaaaa'm'",
                      )}
                    </TimelineOppositeContent>
                    <TimelineSeparator>
                      <TimelineDot
                        sx={{
                          // backgroundColor: theme.palette.error.dark,
                          margin: '0.5px',
                        }}
                        color={colorDot(eventsStates)}
                      />
                      <TimelineConnector />
                    </TimelineSeparator>
                    <TimelineContent>
                      <TreeView
                        aria-label="file system navigator"
                        defaultCollapseIcon={<ExpandMoreIcon />}
                        defaultExpandIcon={<ChevronRightIcon />}
                        key={`tree-item-${id}`}
                      >
                        {eventIds.map((idx) => {
                          return nestedEvents(eventsStates, parseInt(idx));
                        })}
                      </TreeView>
                    </TimelineContent>
                  </TimelineItem>
                ) : (
                  <Typography align="center" sx={{ padding: theme.spacing(1) }} fontWeight="bold">
                    No Event Logs
                  </Typography>
                )}
              </React.Fragment>
            );
          })
        ) : (
          <div>
            <Typography align="center" sx={{ padding: theme.spacing(1) }} fontWeight="bold">
              No Logs to be shown
            </Typography>
          </div>
        )}
      </Timeline>
    </Paper>
  );
}
