import React from 'react';
import { makeStyles, Tooltip } from '@material-ui/core';

export interface TooltipProps {
  title: string;
  id: string;
  children: React.ReactElement;
  enabled: boolean;
}

export default function DashboardTooltip(props: TooltipProps): React.ReactElement {
  const { title, id, enabled } = props;
  const classes = useStyles();

  return (
    <div>
      {enabled && (
        <Tooltip title={title} arrow id={id} className={classes.tooltipWidth} data-testid="tooltip">
          {props.children}
        </Tooltip>
      )}
      {!enabled && props.children}
    </div>
  );
}
const useStyles = makeStyles((_theme) => ({
  tooltipWidth: {
    maxWidth: 200,
  },
}));
