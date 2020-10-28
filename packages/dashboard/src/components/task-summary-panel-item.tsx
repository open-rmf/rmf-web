import React from 'react';
import Debug from 'debug';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { Divider, makeStyles, Typography } from '@material-ui/core';
import TaskManager from '../managers/task-manager';
const debug = Debug('OmniPanel:TaskSummary');

interface TaskSummaryPanelItemProps {
  task: RomiCore.TaskSummary;
}

export const TaskSummaryPanelItem = (props: TaskSummaryPanelItemProps) => {
  debug('render');
  const { task } = props;
  const classes = useStyles();

  const statusDetails = TaskManager.formatStatus(task.status);
  const stateLabel = TaskManager.getStateLabel(task.state);

  return (
    <>
      <div className={classes.accordionDetailLine}>
        <Typography variant="body1">TaskId: </Typography>
        <Typography variant="body1" noWrap>
          {task.task_id}
        </Typography>
      </div>
      <Divider />

      <div className={classes.accordionDetailLine}>
        <Typography variant="body1">State:</Typography>
        <Typography variant="body1">{stateLabel}</Typography>
      </div>
      <Divider />

      <div className={classes.accordionDetailLine}>
        <Typography variant="body1">Status:</Typography>
        <div>
          {statusDetails.map((detail: string) => {
            return (
              <Typography variant="body1" key={detail}>
                {' '}
                {detail}
              </Typography>
            );
          })}
        </div>
      </div>
      <Divider />

      <div className={classes.accordionDetailLine}>
        <Typography variant="body1">Submission Time:</Typography>
        <Typography variant="body1">{task.submission_time.sec}</Typography>
      </div>
      <Divider />

      <div className={classes.accordionDetailLine}>
        <Typography variant="body1">Start time:</Typography>
        <Typography variant="body1">{task.start_time.sec}</Typography>
      </div>
      <Divider />

      <div className={classes.accordionDetailLine}>
        <Typography variant="body1">End Time:</Typography>
        <Typography variant="body1">{task.end_time.sec}</Typography>
      </div>
    </>
  );
};

const useStyles = makeStyles((theme) => ({
  accordionDetailLine: {
    display: 'flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },
}));
