import {
  Button,
  Divider,
  Grid,
  styled,
  Typography,
  useTheme,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
} from '@mui/material';
import HelpIcon from '@mui/icons-material/Help';
import InfoIcon from '@mui/icons-material/Info';
import ErrorIcon from '@mui/icons-material/Error';
import WarningIcon from '@mui/icons-material/Warning';
import BugReportIcon from '@mui/icons-material/BugReport';
import type { TaskState, RobotState, LogEntry } from 'api-client';
import { Tier } from 'api-client';
import { format } from 'date-fns';
import React, { useState } from 'react';
import { CircularProgressBar } from './circular-progress-bar';
import { LinearProgressBar } from './linear-progress-bar';

function getTaskStatusDisplay(assignedTask?: string, taskStatus?: string) {
  if (assignedTask && !taskStatus) {
    return 'Unknown';
  }
  if (assignedTask && taskStatus) {
    return taskStatus;
  } else {
    return 'No Task';
  }
}

const classes = {
  button: 'robot-info-button',
};
const StyledDiv = styled('div')(() => ({
  [`& .${classes.button}`]: {
    '&:hover': {
      background: 'none',
      cursor: 'default',
    },
  },
}));

type TaskStatus = Required<TaskState>['status'];
type RobotIssues = Required<RobotState>['issues'];

export interface RobotInfoProps {
  robotName: string;
  battery?: number;
  assignedTask?: string;
  taskStatus?: TaskStatus;
  taskProgress?: number;
  estFinishTime?: number;
  robotIssues?: RobotIssues;
  robotLogs?: Array<LogEntry>;
}

const finishedStatus: TaskStatus[] = ['failed', 'completed', 'skipped', 'killed', 'canceled'];

export function RobotInfo({
  robotName,
  battery,
  assignedTask,
  taskStatus,
  taskProgress,
  estFinishTime,
  robotIssues,
  robotLogs,
}: RobotInfoProps): JSX.Element {
  const theme = useTheme();
  const hasConcreteEndTime = taskStatus && taskStatus in finishedStatus;
  const robotIssuesArray = robotIssues !== undefined ? robotIssues : [];
  const robotLogsArray = robotLogs !== undefined ? robotLogs : [];

  function mapTierColor(tier?: Tier) {
    // TODO(MXG): We should make this color selection consistent with the color
    // selection that's done for task states.
    switch (tier) {
      case Tier.Uninitialized:
      case Tier.Error:
        return theme.palette.error.dark;

      case Tier.Info:
        return theme.palette.info.light;

      case Tier.Warning:
        return theme.palette.warning.main;

      default:
        return theme.palette.error.dark;
    }
  }

  return (
    <StyledDiv>
      <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
        {robotName}
      </Typography>
      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>
      <Grid container direction={'row'}>
        <Grid container item xs={12} justifyContent="center">
          <Typography variant="h6" gutterBottom sx={{ textTransform: 'capitalize' }}>
            {`Task Progress - ${getTaskStatusDisplay(assignedTask, taskStatus)}`}
          </Typography>
        </Grid>
        <Grid item xs={12}>
          {taskProgress && <LinearProgressBar value={taskProgress * 100} />}
        </Grid>
        <Grid container item xs={12} justifyContent="center">
          <Typography variant="h6" gutterBottom>
            Assigned Tasks
          </Typography>
        </Grid>
        <Grid container item xs={12} justifyContent="center">
          <Button
            disableElevation
            variant="outlined"
            className={classes.button}
            disableRipple={true}
            component="div"
          >
            {assignedTask || '-'}
          </Button>
        </Grid>
        <Grid item xs={6}>
          <Typography variant="h6" align="left">
            Battery
          </Typography>
        </Grid>
        <Grid item xs={6}>
          <Typography variant="h6" align="left">
            <span>{!hasConcreteEndTime && 'Est. '}End Time</span>
          </Typography>
        </Grid>
        <Grid item xs={6}>
          <CircularProgressBar progress={battery ? battery * 100 : 0} strokeColor="#20a39e">
            <Typography variant="h6">{`${battery ? battery * 100 : 0}%`}</Typography>
          </CircularProgressBar>
        </Grid>
        <Grid item xs={6}>
          <Button
            size="small"
            disableElevation
            variant="outlined"
            className={classes.button}
            disableRipple={true}
          >
            {estFinishTime !== undefined ? `${new Date(estFinishTime).toLocaleString()}` : '-'}
          </Button>
        </Grid>
      </Grid>

      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>

      <Grid container direction={'row'}>
        <Grid container item xs={12} justifyContent="left">
          <Typography variant="h6" gutterBottom>
            Logs
          </Typography>
        </Grid>
        <Grid container item xs={12} justifyContent="left">
          <List
            dense
            disablePadding
            component="div"
            role="list"
            sx={{ overflow: 'auto', maxHeight: 250, width: '100%' }}
          >
            {robotLogsArray.map((log, i) => (
              <ListItem key={i} style={{ backgroundColor: mapTierColor(log.tier) }}>
                <ListItemIcon>
                  {log.tier === Tier.Info && <InfoIcon />}
                  {log.tier === Tier.Warning && <WarningIcon />}
                  {log.tier === Tier.Error && <ErrorIcon />}
                  {log.tier === Tier.Uninitialized && <HelpIcon />}
                </ListItemIcon>
                <ListItemText>
                  {format(new Date(log.unix_millis_time), "hh:mm:ss aaaaa'm'")}: {log.text}
                </ListItemText>
              </ListItem>
            ))}
            {robotLogsArray.length === 0 && (
              <Typography gutterBottom justifyContent="center">
                No logs available.
              </Typography>
            )}
          </List>
        </Grid>
      </Grid>

      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>

      <Grid container direction={'row'}>
        <Grid container item xs={12} justifyContent="left">
          <Typography variant="h6" gutterBottom>
            Issues
          </Typography>
        </Grid>
        <Grid container item xs={12} justifyContent="left">
          <List
            dense
            disablePadding
            component="div"
            role="list"
            sx={{ overflow: 'auto', maxHeight: 250, width: '100%' }}
          >
            {robotIssuesArray.map((issue, i) => (
              <ListItem key={i}>
                <ListItemIcon>
                  <BugReportIcon />
                </ListItemIcon>
                <ListItemText>
                  {issue.category}: {JSON.stringify(issue.detail)}
                </ListItemText>
              </ListItem>
            ))}
            {robotIssuesArray.length === 0 && (
              <Typography gutterBottom justifyContent="center">
                No pending issues.
              </Typography>
            )}
          </List>
        </Grid>

        <Divider />
        <div style={{ marginBottom: theme.spacing(1) }}></div>
      </Grid>
    </StyledDiv>
  );
}
