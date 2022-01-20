import { Divider, Grid, Paper, PaperProps, styled, Typography, useTheme } from '@mui/material';
import { TaskEventLog, TaskState } from 'api-client';
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

import { Dialog } from '@mui/material';

import { format } from 'date-fns';
import React from 'react';
const prefix = 'task-logs';
const classes = {
  root: `${prefix}-root`,
};

interface TaskLogProps {
  taskLog: TaskEventLog;
  taskState: TaskState;
  fetchTaskLogs?: () => Promise<never[] | undefined>;
}

export function TaskProgress(props: TaskLogProps) {
  const { taskLog } = props;
  const { taskState } = props;
  console.log(taskState);
  const theme = useTheme();
  const phaseIds = taskLog.phases ? Object.keys(taskLog.phases) : [];
  return (
    <>
      <Divider />
      <Timeline position="right">
        {phaseIds.length > 0 ? (
          phaseIds.map((id: string) => {
            const getEventObj: any = taskLog.phases ? taskLog.phases[id] : null;
            const phaseStateObj: any = taskState.phases ? taskState.phases[id] : null;
            const eventsLogs = getEventObj ? getEventObj['events'] : {};
            const eventsStates = phaseStateObj ? phaseStateObj.events : {};
            const eventIds = eventsLogs ? Object.keys(eventsLogs) : [];
            return (
              <Paper
                sx={{ padding: theme.spacing(1), height: 'inherit' }}
                variant="outlined"
                key={`Phase - ${id}`}
              >
                <Typography variant="h6" fontWeight="bold">
                  {`${phaseStateObj.category} Sequence`}
                </Typography>
                <Divider />
                {eventIds.length > 0 ? (
                  eventIds.map((idx) => {
                    const event = eventsLogs[idx];
                    const stateEvent = eventsStates[idx];
                    return (
                      <TimelineItem>
                        {event.map((e: any, i: any) => {
                          {
                            format(new Date(e.unix_millis_time * 1000), "hh:mm aaaaa'm'");
                          }
                        })}
                        <TimelineOppositeContent color="text.secondary">
                          {format(
                            new Date(phaseStateObj.original_estimate_millis * 1000),
                            "hh:mm aaaaa'm'",
                          )}
                        </TimelineOppositeContent>
                        <TimelineSeparator>
                          <TimelineDot />
                          <TimelineConnector />
                        </TimelineSeparator>
                        <TimelineContent>
                          {/* {event.map((e: any, i: any) => { */}
                          <TreeView
                            aria-label="file system navigator"
                            defaultCollapseIcon={<ExpandMoreIcon />}
                            defaultExpandIcon={<ChevronRightIcon />}
                          >
                            <TreeItem nodeId="1" label={'event.text'}>
                              <TreeItem nodeId="2" label="Calendar" />
                            </TreeItem>
                            <TreeItem nodeId="5" label="Documents">
                              <TreeItem nodeId="10" label="OSS" />
                              <TreeItem nodeId="6" label="MUI">
                                <TreeItem nodeId="8" label="index.js" />
                              </TreeItem>
                            </TreeItem>
                          </TreeView>
                          {/* })} */}
                        </TimelineContent>
                      </TimelineItem>
                    );
                  })
                ) : (
                  <Typography align="center" sx={{ padding: theme.spacing(1) }} fontWeight="bold">
                    No Event Logs
                  </Typography>
                )}
              </Paper>
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
