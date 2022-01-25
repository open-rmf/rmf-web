import { Tooltip as MuiTooltip, styled } from '@mui/material';
import React from 'react';

export interface TooltipProps {
  title: string;
  id: string;
  enabled: boolean; //this prop allows you to enabled / disable the use of tooltips
  children: JSX.Element;
}

const classes = {
  tooltipWidth: 'tooltip-width',
};
const StyledDiv = styled('div')(() => ({
  [`& .${classes.tooltipWidth}`]: {
    maxWidth: 200,
  },
}));

export const Tooltip = (props: TooltipProps): JSX.Element => {
  const { title, id, enabled } = props;
  return (
    <StyledDiv>
      {enabled && (
        <MuiTooltip
          title={title}
          arrow
          id={id}
          className={classes.tooltipWidth}
          data-testid={id + '-tooltip'}
        >
          {props.children}
        </MuiTooltip>
      )}
      {!enabled && props.children}
    </StyledDiv>
  );
};

export default Tooltip;
