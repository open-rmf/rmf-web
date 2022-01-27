import { styled, Typography, useTheme } from '@mui/material';
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
const prefix = 'task-logs';
const classes = {
  root: `${prefix}-root`,
};

interface TaskLogProps {
  taskLog: TaskEventLog;
  taskState: TaskState;
  fetchTaskLogs?: () => Promise<never[] | undefined>;
}

function nestedEvents(eventsStates: EventState[], child: number) {
  const deps = eventsStates[child] ? eventsStates[child].deps : [];
  return (
    <TreeItem nodeId={`parent-${child}`} label={eventsStates[child].name}>
      {deps?.map((id) => {
        if (eventsStates[id].deps?.length) {
          nestedEvents(eventsStates, id);
        } else {
          return <TreeItem nodeId={`child-${id}`} label={eventsStates[id].name}></TreeItem>;
        }
      })}
    </TreeItem>
  );
}

function colorDot(eventsStates: EventState[]) {
  for (const id in eventsStates) {
    const event: EventState = eventsStates[id];
    if (event.status === 'underway') {
      return 'info';
    }
    if (event.status === 'standby') {
      return 'warning';
    }
  }
  return 'success' as const;
}

export function TaskProgress(props: TaskLogProps) {
  const { taskLog } = props;
  const { taskState } = props;
  const theme = useTheme();
  const phaseIds = taskLog.phases ? Object.keys(taskLog.phases) : [];

  return (
    <>
      <Timeline position="right">
        {phaseIds.length > 0 ? (
          phaseIds.map((id: string) => {
            const getEventObj: any = taskLog.phases ? taskLog.phases[id] : null;
            const eventsLogs = getEventObj ? getEventObj['events'] : {};
            const phaseStateObj: any = taskState.phases ? taskState.phases[id] : null;
            const eventsStates = phaseStateObj ? phaseStateObj.events : {};
            const eventIds = eventsStates ? Object.keys(eventsStates) : [];

            return (
              <>
                {eventIds.length > 0 ? (
                  <TimelineItem>
                    <TimelineOppositeContent color="text.secondary">
                      {format(
                        new Date(phaseStateObj.original_estimate_millis * 1000),
                        "hh:mm aaaaa'm'",
                      )}
                    </TimelineOppositeContent>
                    <TimelineSeparator>
                      <TimelineDot color={colorDot(eventsStates)} />
                      <TimelineConnector />
                    </TimelineSeparator>
                    <TimelineContent>
                      <TreeView
                        aria-label="file system navigator"
                        defaultCollapseIcon={<ExpandMoreIcon />}
                        defaultExpandIcon={<ChevronRightIcon />}
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
              </>
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
    </>
  );
}
