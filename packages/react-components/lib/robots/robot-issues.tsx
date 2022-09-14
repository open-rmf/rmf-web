import { Accordion, AccordionDetails, AccordionSummary, Box, Typography } from '@mui/material';
import ErrorOutlineOutlinedIcon from '@mui/icons-material/ErrorOutlineOutlined';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import type { RobotState } from 'api-client';
import React from 'react';

type RobotIssues = Required<RobotState>['issues'];

export interface RobotIssuesProps {
  robotIssues?: RobotIssues;
}

export function RobotIssues({ robotIssues }: RobotIssuesProps): JSX.Element {
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
        <Accordion key={i}>
          <AccordionSummary expandIcon={<ExpandMoreIcon />} id={String(i)}>
            <ErrorOutlineOutlinedIcon style={{ minWidth: '40px' }} />
            {issue.category}
          </AccordionSummary>
          <AccordionDetails>
            <Typography variant="subtitle2">{JSON.stringify(issue.detail, null, 2)}</Typography>
          </AccordionDetails>
        </Accordion>
      ))}
    </div>
  );
}
