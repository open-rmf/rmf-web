import { makeStyles, Tooltip } from '@material-ui/core';
import React from 'react';

export interface TooltipProps {
  title: string;
  id: string;
  enabled: boolean; //this prop allows you to enabled / disable the use of tooltips
  children: JSX.Element;
}

const useStyles = makeStyles({
  tooltipWidth: {
    maxWidth: 200,
  },
});

export const DashboardTooltip = (props: TooltipProps): JSX.Element => {
  const { title, id, enabled } = props;
  const classes = useStyles();
  return (
    <div>
      {enabled && (
        <Tooltip
          title={title}
          arrow
          id={id}
          className={classes.tooltipWidth}
          data-testid={id + '-tooltip'}
        >
          {props.children}
        </Tooltip>
      )}
      {!enabled && props.children}
    </div>
  );
};

export default DashboardTooltip;
