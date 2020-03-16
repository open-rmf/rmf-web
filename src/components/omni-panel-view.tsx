import React from 'react';
import { makeStyles } from '@material-ui/core';

const useStyles = makeStyles(() => ({
  container: {
    overflow: 'auto',
  },
}));

export interface OmniPanelViewProps {
  value: number;
  index: number;
  children: React.ReactNode;
}

export const OmniPanelView = React.forwardRef(
  (props: OmniPanelViewProps, ref: React.Ref<HTMLDivElement>) => {
    const classes = useStyles();
    const { value, index, children } = props;
    return (
      <div ref={ref} className={classes.container}>
        {value === index && children}
      </div>
    );
  },
);

export default OmniPanelView;
