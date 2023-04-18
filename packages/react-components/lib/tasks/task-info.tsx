import { Divider, Typography, useTheme } from '@mui/material';
import { styled } from '@mui/material';
import type { TaskState } from 'api-client';
import React from 'react';
import { TaskTimeline } from './task-timeline';

const classes = {
  infoValue: 'task-info-info-value',
};
const StyledDiv = styled('div')(() => ({
  [`& .${classes.infoValue}`]: {
    float: 'right',
    textAlign: 'right',
  },
}));

function InfoLine({ children }: React.PropsWithChildren<unknown>) {
  return (
    <Typography variant="body1" gutterBottom>
      {children}
    </Typography>
  );
}

function InfoValue({ children }: React.PropsWithChildren<unknown>) {
  return <span className={classes.infoValue}>{children}</span>;
}

export interface TaskInfoProps {
  task: TaskState;
  title?: string;
}

export function TaskInfo({ task, title }: TaskInfoProps): JSX.Element {
  const theme = useTheme();

  const [startWaypoint, setStartWaypoint] = React.useState('');
  const [finishWaypoint, setFinishWaypoint] = React.useState('');

  React.useEffect(() => {
    if (task.phases) {
      /**
       * We assumed that phases[index].detail always return
       * "Go to " + goal_name_,
          "Moving the robot from " + start_name + " to " + goal_name_,
          *estimate);

          According with this code
       * https://github.com/open-rmf/rmf_task/blob/main/rmf_task_sequence/src/rmf_task_sequence/events/GoToPlace.cpp#L252
       */
      const firstPhase = Object.values(task.phases)[0]?.detail;

      if (!firstPhase) {
        setStartWaypoint('');
        return;
      }

      const colonIndex = firstPhase.toString().indexOf(':');
      const closingBracketIndex = firstPhase.toString().indexOf(']');

      setStartWaypoint(firstPhase.toString().substring(colonIndex + 1, closingBracketIndex));

      const lastPhase = Object.values(task.phases).pop()?.detail;
      if (lastPhase) {
        setFinishWaypoint(lastPhase.toString().replace(/^.+:/, '').replace(/.$/, ''));
      }
    }
  }, [task]);

  return (
    <StyledDiv>
      <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
        {title ? title : task.booking.id}
      </Typography>
      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>
      <InfoLine>
        <span>Status:</span>
        <InfoValue>{task.status || 'unknown'}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Start Waypoint:</span>
        <InfoValue>{startWaypoint}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Finish Waypoint</span>
        <InfoValue>{finishWaypoint}</InfoValue>
      </InfoLine>
      <div style={{ padding: '4px' }}>
        <TaskTimeline taskState={task} />
      </div>
    </StyledDiv>
  );
}
