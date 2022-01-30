import { Divider, Grid, Paper, PaperProps, styled, Typography, useTheme } from '@mui/material';
import { TaskEventLog, TaskState, LogEntry } from 'api-client';
import { format } from 'date-fns';
import React from 'react';
import TimelineDot from '@mui/lab/TimelineDot';
const prefix = 'task-logs';
const classes = {
  root: `${prefix}-root`,
};

interface TaskLogProps {
  taskLog: TaskEventLog;
  taskState: TaskState;
  fetchTaskLogs?: () => Promise<never[] | undefined>;
}

const StyledPaper = styled((props: PaperProps) => <Paper variant="outlined" {...props} />)(
  ({ theme }) => ({
    [`&.${classes.root}`]: {
      padding: theme.spacing(1),
      width: '95%',
      flex: '0 0 auto',
      maxHeight: '95%',
      overflow: 'auto',
    },
  }),
);

export function TaskLogs(props: TaskLogProps) {
  const { taskLog } = props;
  const { taskState } = props;
  const theme = useTheme();
  const phaseIds = taskLog.phases ? Object.keys(taskLog.phases) : [];

  function MapEventColor(event: LogEntry) {
    switch (event.tier) {
      case 'warning':
        return theme.palette.warning.main;

      case 'error':
        return theme.palette.error.main;

      default:
        return theme.palette.info.main;
    }
  }
  return (
    <StyledPaper className={classes.root}>
      <Typography variant="h5" style={{ textAlign: 'center' }} gutterBottom>
        {taskLog.task_id}
      </Typography>
      <Divider />
      {phaseIds.length > 0 ? (
        phaseIds.map((id: string) => {
          const getEventObj: any = taskLog.phases ? taskLog.phases[id] : null;
          const events = getEventObj ? getEventObj['events'] : {};
          const eventIds = events ? Object.keys(events) : [];
          const phaseStateObj: any = taskState.phases ? taskState.phases[id] : null;
          const eventsStates = phaseStateObj ? phaseStateObj.events : {};

          return (
            <>
              <Typography variant="h6" fontWeight="bold" marginTop={3}>
                {phaseStateObj.category}
              </Typography>
              <Divider />
              {eventIds.length > 0 ? (
                eventIds.map((idx) => {
                  const event = events[idx];
                  return (
                    <div
                      style={{
                        marginTop: theme.spacing(1),
                        backgroundColor: MapEventColor(event),
                        padding: theme.spacing(1),
                        borderRadius: '6px',
                      }}
                      key={`event - ${idx}`}
                    >
                      <Typography variant="body1" fontWeight="bold">
                        {eventsStates && eventsStates.length > 0 ? eventsStates[0].name : null}
                      </Typography>
                      {event.map((e: any, i: any) => {
                        return (
                          <Grid
                            container
                            key={`info-${i}`}
                            direction="row"
                            justifyItems="center"
                            sx={{
                              backgroundColor: 'white',
                              marginTop: theme.spacing(1),
                              borderRadius: '8px',
                            }}
                          >
                            <Grid
                              item
                              xs={4}
                              sx={{
                                border: 1,
                                padding: theme.spacing(1),
                              }}
                            >
                              <Typography variant="body1">
                                {format(new Date(e.unix_millis_time * 1000), "hh:mm aaaaa'm'")}
                              </Typography>
                            </Grid>
                            <Grid
                              item
                              xs={8}
                              sx={{
                                padding: theme.spacing(1),
                                borderRight: 1,
                                borderBottom: 1,
                                borderTop: 1,
                              }}
                            >
                              <Typography variant="body1">{e.text}</Typography>
                            </Grid>
                          </Grid>
                        );
                      })}
                    </div>
                  );
                })
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
    </StyledPaper>
  );
}
