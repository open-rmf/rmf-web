import {
  Accordion,
  AccordionDetails,
  AccordionSummary,
  Box,
  Typography,
  useTheme,
} from '@mui/material';
import BugReportIcon from '@mui/icons-material/BugReport';
import CheckCircleIcon from '@mui/icons-material/CheckCircle';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import type { RobotState } from 'api-client';
import React from 'react';

type RobotIssues = Required<RobotState>['issues'];

export interface RobotIssuesProps {
  robotIssues?: RobotIssues;
}

export function RobotIssues({ robotIssues }: RobotIssuesProps): JSX.Element {
  const theme = useTheme();
  const robotIssuesArray = robotIssues !== undefined ? robotIssues : [];
  if (robotIssuesArray.length === 0) {
    return (
      <Box sx={{ display: 'flex', height: '100%', alignItems: 'center', justifyContent: 'center' }}>
        <Typography variant="h6" align="center">
          No issues reported.
        </Typography>
      </Box>
    );
  }
  return (
    <div>
      {robotIssuesArray.map((issue, i) => (
        <Accordion>
          <AccordionSummary expandIcon={<ExpandMoreIcon />} id={String(i)}>
            <BugReportIcon />
            {issue.category}
          </AccordionSummary>
          <AccordionDetails>
            <Typography>{JSON.stringify(issue.detail)}</Typography>
          </AccordionDetails>
        </Accordion>
      ))}
    </div>
  );
}
