import { Divider, Grid, Paper, PaperProps, styled, Typography, useTheme } from '@mui/material';
import type { EventState, TaskEventLog } from 'api-client';
import React from 'react';

type EventStatus = Required<EventState>['status'];

const prefix = 'task-logs';
const classes = {
  root: `${prefix}-root`,
};

export interface TaskLogsProps {
  taskLog: TaskEventLog;
  eventName: (phaseId: string, eventId: string) => string;
  eventStatus: (phaseId: string, eventId: string) => EventStatus | undefined;
}

const StyledPaper = styled((props: PaperProps) => <Paper variant="outlined" {...props} />)(
  ({ theme }) => ({
    [`&.${classes.root}`]: {
      padding: theme.spacing(1),
      width: '100%',
      flex: '0 0 auto',
      maxHeight: '100%',
      overflow: 'auto',
    },
  }),
);

export const TaskLogs: React.FC<TaskLogsProps> = ({ taskLog, eventName, eventStatus }) => {
  const theme = useTheme();

  function mapEventColor(eventStatus?: EventStatus) {
    // TODO(MXG): We should make this color selection consistent with the color
    // selection that's done for task states.
    if (eventStatus == null) return theme.palette.warning.light;

    switch (eventStatus) {
      case 'uninitialized':
      case 'blocked':
      case 'error':
      case 'failed':
        return theme.palette.error.dark;

      case 'queued':
      case 'standby':
        return theme.palette.info.light;

      case 'underway':
        return theme.palette.success.light;

      case 'delayed':
        return theme.palette.warning.main;

      case 'skipped':
      case 'canceled':
      case 'killed':
        return theme.palette.error.light;

      case 'completed':
        return theme.palette.info.light;

      default:
        return theme.palette.error.dark;
    }
  }

  return (
    <StyledPaper className={classes.root}>
      <Typography variant="h5" style={{ textAlign: 'center' }} gutterBottom>
        {taskLog.task_id}
      </Typography>
      <Divider />
      {taskLog.phases ? (
        Object.entries(taskLog.phases).map(([phaseId, phase]) => (
          <Paper sx={{ padding: theme.spacing(1) }} variant="outlined" key={phaseId}>
            <Typography variant="h6" fontWeight="bold" marginTop={3}>
              Phase - {phaseId}
            </Typography>

            <Divider />
            {phase.events ? (
              Object.entries(phase.events).map(([eventId, event]) => {
                return (
                  <div
                    style={{
                      marginTop: theme.spacing(1),
                      backgroundColor: mapEventColor(eventStatus(phaseId, eventId)),
                      padding: theme.spacing(1),
                      borderRadius: theme.spacing(1),
                    }}
                    key={eventId}
                  >
                    <Typography variant="body1" fontWeight="bold">
                      {eventName(phaseId, eventId)}
                    </Typography>
                    {event.length > 0 ? (
                      event.map((log, idx) => (
                        <Grid
                          container
                          key={idx}
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
                              padding: theme.spacing(1),
                            }}
                          >
                            <Typography variant="body1">
                              {new Date(log.unix_millis_time).toLocaleString()}
                            </Typography>
                          </Grid>
                          <Grid
                            item
                            xs={8}
                            sx={{
                              padding: theme.spacing(1),
                            }}
                          >
                            <Typography variant="body1">{log.text}</Typography>
                          </Grid>
                        </Grid>
                      ))
                    ) : (
                      <Typography
                        align="center"
                        sx={{ padding: theme.spacing(1) }}
                        fontWeight="bold"
                      >
                        No Logs
                      </Typography>
                    )}
                  </div>
                );
              })
            ) : (
              <Typography align="center" sx={{ padding: theme.spacing(1) }} fontWeight="bold">
                No Event Logs
              </Typography>
            )}
          </Paper>
        ))
      ) : (
        <div>
          <Typography align="center" sx={{ padding: theme.spacing(1) }} fontWeight="bold">
            No logs to be shown
          </Typography>
        </div>
      )}
    </StyledPaper>
  );
};
