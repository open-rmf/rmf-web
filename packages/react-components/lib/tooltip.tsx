import { makeStyles, Tooltip, Typography } from '@material-ui/core';
import React from 'react';

export interface TooltipProps {
  title: string;
  id: string;
  children: JSX.Element;
}

const useStyles = makeStyles((theme) => ({
  tooltipWidth: {
    maxWidth: 200,
  },
}));

export const DashboardTooltip = (props: TooltipProps): JSX.Element => {
  const { title, id } = props;
  const classes = useStyles();
  return (
    <div>
      <Tooltip title={title} arrow id={id} className={classes.tooltipWidth} data-testid="tooltip">
        <Typography variant="h5">Hover over me</Typography>
      </Tooltip>
    </div>
  );
};

export default DashboardTooltip;
