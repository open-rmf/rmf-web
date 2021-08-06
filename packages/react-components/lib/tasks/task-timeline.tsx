import React from 'react';
import { makeStyles, Theme, useTheme } from '@material-ui/core/styles';
import {
  Timeline,
  TimelineItem,
  TimelineSeparator,
  TimelineConnector,
  TimelineContent,
  TimelineOppositeContent,
  TimelineDot,
} from '@material-ui/lab';
import Paper from '@material-ui/core/Paper';
import Typography from '@material-ui/core/Typography';
import * as RmfModels from 'rmf-models';
import { rosTimeToJs } from '../utils';

const getPhaseColors = (theme: Theme) => ({
  pending: theme.palette.info.light,
  completed: theme.palette.success.light,
  failed: theme.palette.error.light,
});

const useStyles = makeStyles((theme) => {
  const phaseColors = getPhaseColors(theme);
  return {
    paper: {
      padding: '6px 16px',
    },
    secondaryTail: {
      backgroundColor: theme.palette.secondary.main,
    },
    pendingPhase: {
      background: phaseColors.pending,
    },
    completedPhase: {
      background: phaseColors.completed,
    },
    failedPhase: {
      background: phaseColors.failed,
    },
  };
});

export interface TaskTimelineProps {
  taskSummary: RmfModels.TaskSummary;
}

export function TaskTimeline({ taskSummary }: TaskTimelineProps): JSX.Element {
  const classes = useStyles();
  const theme = useTheme();
  const phaseColors = getPhaseColors(theme);
  const timelinePhases = taskSummary.status.split('\n\n');
  const currentDotIdx = timelinePhases.findIndex((msg) => msg.startsWith('*'));
  const timelineInfo = taskSummary.status.split('\n\n');

  const timelineDotProps = timelinePhases.map((_, idx) => {
    if (
      [RmfModels.TaskSummary.STATE_CANCELED, RmfModels.TaskSummary.STATE_FAILED].includes(
        taskSummary.state,
      )
    ) {
      return {
        className: classes.failedPhase,
        color: phaseColors.failed,
      };
    }

    if (taskSummary.state === RmfModels.TaskSummary.STATE_COMPLETED) {
      return {
        className: classes.completedPhase,
        color: phaseColors.completed,
      };
    }

    if (taskSummary.state === RmfModels.TaskSummary.STATE_ACTIVE && idx < currentDotIdx) {
      return {
        className: classes.completedPhase,
        color: phaseColors.completed,
      };
    }

    return {
      className: classes.pendingPhase,
      color: phaseColors.pending,
    };
  });

  return (
    <Timeline>
      {timelineInfo.map((dotInfo, idx) => {
        return (
          <TimelineItem key={idx}>
            <TimelineOppositeContent>
              <Typography variant="body2" color="textSecondary">
                {idx == 0 && rosTimeToJs(taskSummary.start_time).toLocaleString()}
                {idx == timelineInfo.length - 1 &&
                  rosTimeToJs(taskSummary.end_time).toLocaleString()}
              </Typography>
            </TimelineOppositeContent>
            <TimelineSeparator>
              <TimelineDot className={timelineDotProps[idx].className} />
              <TimelineConnector />
            </TimelineSeparator>
            <TimelineContent>
              <Paper elevation={3} style={{ padding: '6px 16px' }}>
                <Typography>{dotInfo}</Typography>
              </Paper>
            </TimelineContent>
          </TimelineItem>
        );
      })}
    </Timeline>
  );
}
