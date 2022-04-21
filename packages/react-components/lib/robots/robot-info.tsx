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
  // Collapse,
} from '@mui/material';
// import ExpandLess from '@mui/icons-material/ExpandLess';
// import ExpandMore from '@mui/icons-material/ExpandMore';
import ReportProblemRoundedIcon from '@mui/icons-material/ReportProblemRounded';
import type { TaskState, RobotState } from 'api-client';
// import React, { useState } from 'react';
import React from 'react';
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

// const ExpandableIssue = (category: string, detail: string): JSX.Element => {
//   const [open, setOpen] = useState(false);
//   const handleClick = () => {
//     setOpen(!open);
//   };

//   return (
//     <div>
//       <ListItem button onClick={handleClick}>
//         <ListItemIcon>
//           <ReportProblemRoundedIcon />
//         </ListItemIcon>
//         <ListItemText>{category}</ListItemText>
//         {open ? <ExpandLess /> : <ExpandMore />}
//       </ListItem>
//       <Collapse in={open} timeout="auto" unmountOnExit>
//         <ListItem>
//           <ListItemText>{detail}</ListItemText>
//         </ListItem>
//       </Collapse>
//     </div>
//   );
// };

export interface RobotInfoProps {
  robotName: string;
  battery?: number;
  assignedTask?: string;
  taskStatus?: TaskStatus;
  taskProgress?: number;
  estFinishTime?: number;
  robotIssues?: RobotIssues;
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
}: RobotInfoProps): JSX.Element {
  const theme = useTheme();
  const hasConcreteEndTime = taskStatus && taskStatus in finishedStatus;
  const robotIssuesArray = robotIssues !== undefined ? robotIssues : [];

  return (
    <StyledDiv>
      <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
        {robotName}
      </Typography>
      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>
      <Grid container>
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

        <Divider />

        <Grid container item xs={12} justifyContent="center">
          <Typography variant="h6" gutterBottom>
            Issues
          </Typography>
        </Grid>
        <Grid container item xs={12} justifyContent="center">
          <List dense disablePadding component="div" role="list">
            {robotIssuesArray.map((issue, i) => (
              // <ExpandableIssue key={i}
              //   category={issue.category !== undefined ? issue.category : 'unnamed category'}
              //   detail={JSON.stringify(issue.detail)} />
              <ListItem key={i}>
                <ListItemIcon>
                  <ReportProblemRoundedIcon />
                </ListItemIcon>
                <ListItemText>{JSON.stringify(issue)}</ListItemText>
              </ListItem>
            ))}
            {robotIssuesArray.length === 0 && (
              <Typography gutterBottom>No pending issues.</Typography>
            )}
          </List>
        </Grid>
      </Grid>
    </StyledDiv>
  );
}
