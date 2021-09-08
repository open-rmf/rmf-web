import Paper from '@mui/material/Paper';
import { Theme } from '@mui/material/styles';
import { makeStyles } from '@mui/styles';
import Typography from '@mui/material/Typography';
import {
  Timeline,
  TimelineConnector,
  TimelineContent,
  TimelineDot,
  TimelineItem,
  TimelineOppositeContent,
  TimelineSeparator,
} from '@mui/lab';
import React from 'react';
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
      width: '200px',
      maxHeight: '100px',
      overflow: 'auto',
      display: 'inline-block',
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
    timelineRoot: {
      padding: '6px 0px',
    },
  };
});

export interface TaskTimelineProps {
  taskSummary: RmfModels.TaskSummary;
}

export function TaskTimeline({ taskSummary }: TaskTimelineProps): JSX.Element {
  const classes = useStyles();
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
      };
    }

    if (taskSummary.state === RmfModels.TaskSummary.STATE_COMPLETED) {
      return {
        className: classes.completedPhase,
      };
    }

    if (taskSummary.state === RmfModels.TaskSummary.STATE_ACTIVE && idx < currentDotIdx) {
      return {
        className: classes.completedPhase,
      };
    }

    return {
      className: classes.pendingPhase,
    };
  });

  return (
    <Timeline align="left" className={classes.timelineRoot}>
      {timelineInfo.map((dotInfo, idx) => {
        return (
          <TimelineItem key={idx}>
            <TimelineOppositeContent style={{ flex: 0.1, padding: '0px 12px 0px 0px' }}>
              <Typography variant="overline" color="textSecondary" style={{ textAlign: 'justify' }}>
                {idx === 0 && rosTimeToJs(taskSummary.start_time).toLocaleTimeString()}
                {idx > 0 &&
                  idx === timelineInfo.length - 1 &&
                  rosTimeToJs(taskSummary.end_time).toLocaleTimeString()}
              </Typography>
            </TimelineOppositeContent>
            <TimelineSeparator>
              <TimelineDot {...timelineDotProps[idx]} />
              {idx < timelineInfo.length - 1 && <TimelineConnector />}
            </TimelineSeparator>
            <TimelineContent>
              <Paper className={classes.paper}>
                <Typography variant="caption">{dotInfo}</Typography>
              </Paper>
            </TimelineContent>
          </TimelineItem>
        );
      })}
    </Timeline>
  );
}
