import React from 'react';
import { SpotlightValue } from './spotlight-value';
import Debug from 'debug';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { Button, Divider, makeStyles, Typography } from '@material-ui/core';
import TaskManager from '../managers/task-manager';

const debug = Debug('OmniPanel:NegotiationsPanel');

export interface TaskSummaryPanelProps {
  allTasks: RomiCore.TaskSummary[];
  spotlight?: Readonly<SpotlightValue<string>>;
  transport?: Readonly<RomiCore.Transport>;
}

export const TaskSummaryPanel = React.memo((props: TaskSummaryPanelProps) => {
  debug('task summary status panel render');

  const { allTasks, spotlight } = props;
  const classes = useStyles();

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    // TODO: spotlight
  }, [spotlight]);

  const listItems = allTasks.map((task) => {
    console.log(task);
    const statusDetails = TaskManager.formatStatus(task.status);
    const stateLabel = TaskManager.getStateLabel(task.state);
    return (
      <div key={task.task_id}>
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
            {statusDetails.map((detail) => {
              return (
                <>
                  <Typography variant="body1"> {detail}</Typography>
                  <Divider />
                </>
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
        <div className={classes.buttonWrapper}>
          <Button className={classes.button} variant="contained" color="primary">
            {'Pause'}
          </Button>
        </div>
      </div>
    );
  });

  return <>{listItems}</>;
});

const useStyles = makeStyles((theme) => ({
  accordionDetailLine: {
    display: 'flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },
  button: {
    width: '100%',
  },
  buttonWrapper: {
    padding: '1rem 0.5rem',
  },
}));

export default TaskSummaryPanel;
