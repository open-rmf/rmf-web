import React from 'react';
import Debug from 'debug';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { Button, Divider, IconButton, makeStyles, Typography } from '@material-ui/core';
import TaskManager, { TaskState } from '../managers/task-manager';
import { TreeItem } from '@material-ui/lab';
import { colorPalette } from '../util/css-utils';
// import PauseCircleFilledIcon from '@material-ui/icons/PauseCircleFilled';
import PlayCircleFilledWhiteIcon from '@material-ui/icons/PlayCircleFilledWhite';
const debug = Debug('OmniPanel:NegotiationsPanel');

interface TaskSummaryPanelItemProps {
  task: RomiCore.TaskSummary;
}

export const TaskSummaryPanelItem = (props: TaskSummaryPanelItemProps) => {
  const { task } = props;
  const classes = useStyles();

  const statusDetails = TaskManager.formatStatus(task.status);
  const stateLabel = TaskManager.getStateLabel(task.state);

  const determineStyle = (state: TaskState): string => {
    switch (state) {
      case TaskState.STATE_QUEUED:
        return classes.queued;
      case TaskState.STATE_ACTIVE:
        return classes.active;
      case TaskState.STATE_COMPLETED:
        return classes.completed;
      case TaskState.STATE_FAILED:
        return classes.failed;
      default:
        return 'UNKNOWN';
    }
  };
  console.log(determineStyle(task.state));
  return (
    <TreeItem
      data-component="TreeItem"
      nodeId={task.task_id}
      key={task.task_id}
      classes={{
        label: `${determineStyle(task.state)} ${classes.labelContent}`,
        root: classes.treeChildren,
        expanded: classes.expanded,
      }}
      label={
        <Typography>
          {statusDetails.statusLabel}
          <IconButton
            onClick={(e) => {
              e.preventDefault();
              console.log('Not implemented');
            }}
          >
            <PlayCircleFilledWhiteIcon />
          </IconButton>
        </Typography>
      }
    >
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

      {/* <div className={classes.accordionDetailLine}>
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
        <Divider /> */}

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
    </TreeItem>
  );
};

const useStyles = makeStyles((theme) => ({
  accordionDetailLine: {
    display: 'flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
  },
  treeChildren: {
    margin: '0.5rem 0',
  },
  labelContent: {
    padding: '0.5rem',
    borderRadius: '0.5rem',
    boxShadow: '0 0 25px 0 rgb(72, 94, 116, 0.3)',
  },
  expanded: {
    borderLeft: `0.1rem solid ${colorPalette.unknown}`,
  },
  completed: {
    backgroundColor: 'lightgreen',
  },
  queued: {
    backgroundColor: theme.palette.warning.main,
  },
  active: {
    backgroundColor: 'yellow',
  },
  failed: {
    backgroundColor: theme.palette.error.main,
  },
}));

// const listItems = allTasks.map((task) => {
//   console.log(task);
//   const statusDetails = TaskManager.formatStatus(task.status);
//   const stateLabel = TaskManager.getStateLabel(task.state);
//   return (
//     <div key={task.task_id}>
//       <div className={classes.accordionDetailLine}>
//         <Typography variant="body1">TaskId: </Typography>
//         <Typography variant="body1" noWrap>
//           {task.task_id}
//         </Typography>
//       </div>
//       <Divider />

//       <div className={classes.accordionDetailLine}>
//         <Typography variant="body1">State:</Typography>
//         <Typography variant="body1">{stateLabel}</Typography>
//       </div>
//       <Divider />

//       <div className={classes.accordionDetailLine}>
//         <Typography variant="body1">Status:</Typography>
//         <div>
//           {statusDetails.map((detail) => {
//             return (
//               <>
//                 <Typography variant="body1"> {detail}</Typography>
//                 <Divider />
//               </>
//             );
//           })}
//         </div>
//       </div>
//       <Divider />

//       <div className={classes.accordionDetailLine}>
//         <Typography variant="body1">Submission Time:</Typography>
//         <Typography variant="body1">{task.submission_time.sec}</Typography>
//       </div>
//       <Divider />

//       <div className={classes.accordionDetailLine}>
//         <Typography variant="body1">Start time:</Typography>
//         <Typography variant="body1">{task.start_time.sec}</Typography>
//       </div>
//       <Divider />

//       <div className={classes.accordionDetailLine}>
//         <Typography variant="body1">End Time:</Typography>
//         <Typography variant="body1">{task.end_time.sec}</Typography>
//       </div>
//       <div className={classes.buttonWrapper}>
//         <Button className={classes.button} variant="contained" color="primary">
//           {'Pause'}
//         </Button>
//       </div>
//     </div>
//   );
// });
