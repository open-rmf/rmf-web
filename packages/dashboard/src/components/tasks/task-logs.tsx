import { Box, Divider, Grid, Paper, Typography, useTheme } from '@mui/material';
import {
  EventState,
  ApiServerModelsRmfApiTaskStateStatus as TaskStatus,
  TaskEventLog,
  TaskState,
} from 'api-client';
import { format } from 'date-fns';

interface TaskLogProps {
  taskLog: TaskEventLog | null;
  taskState: TaskState | null;
  fetchTaskLogs?: () => Promise<never[] | undefined>;
  title?: string;
}

export function TaskLogs({ taskLog, taskState, title }: TaskLogProps) {
  const theme = useTheme();
  const phaseIds = taskLog && taskLog.phases ? Object.keys(taskLog.phases) : [];

  function mapEventColor(event: EventState | null) {
    // TODO(MXG): We should make this color selection consistent with the color
    // selection that's done for task states.
    if (event == null || event.status == null) return theme.palette.warning.light;

    switch (event.status) {
      case TaskStatus.Uninitialized:
      case TaskStatus.Blocked:
      case TaskStatus.Error:
      case TaskStatus.Failed:
        return theme.palette.error.dark;

      case TaskStatus.Queued:
      case TaskStatus.Standby:
        return theme.palette.info.light;

      case TaskStatus.Underway:
        return theme.palette.success.light;

      case TaskStatus.Delayed:
        return theme.palette.warning.main;

      case TaskStatus.Skipped:
      case TaskStatus.Canceled:
      case TaskStatus.Killed:
        return theme.palette.error.light;

      case TaskStatus.Completed:
        return theme.palette.info.light;

      default:
        return theme.palette.error.dark;
    }
  }

  return (
    <Box component="div">
      <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
        {taskState && (title ? title : taskState.booking.id)}
      </Typography>
      <Divider />
      {phaseIds.length > 0 ? (
        phaseIds.map((id: string) => {
          const getEventObj: any = taskLog && taskLog.phases ? taskLog.phases[id] : null;
          const events = getEventObj ? getEventObj['events'] : {};
          const eventIds = events ? Object.keys(events) : [];
          const phaseStateObj = taskState && taskState.phases ? taskState.phases[id] : null;
          const eventStates = phaseStateObj ? phaseStateObj.events : {};

          return (
            <Paper sx={{ padding: theme.spacing(1) }} variant="outlined" key={`Phase - ${id}`}>
              <Typography variant="h6" fontWeight="bold" marginTop={3}>
                {phaseStateObj && phaseStateObj.id ? phaseStateObj.id.toString() : 'unknown #'}.{' '}
                {phaseStateObj && phaseStateObj.category ? phaseStateObj.category : 'undefined'}
              </Typography>

              <Divider />
              {eventIds.length > 0 ? (
                eventIds.map((idx) => {
                  const event = events[idx];
                  const eventState = eventStates ? eventStates[idx] : null;
                  return (
                    <div
                      style={{
                        marginTop: theme.spacing(1),
                        backgroundColor: mapEventColor(eventState),
                        padding: theme.spacing(1),
                        borderRadius: theme.spacing(1),
                      }}
                      key={`event - ${idx}`}
                    >
                      <Typography variant="body1" fontWeight="bold">
                        {eventState?.name}
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
                                {format(new Date(e.unix_millis_time), "hh:mm:ss aaaaa'm'")}
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
    </Box>
  );
}
