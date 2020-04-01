import { makeStyles, Slide } from '@material-ui/core';
import React from 'react';

const useStyles = makeStyles(() => ({
  container: {
    width: '100%',
    height: '100%',
    position: 'absolute',
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
    const show = value === index;
    return (
      <Slide
        direction="left"
        in={show}
        mountOnEnter
        unmountOnExit
        appear={false}
      >
        <div className={classes.container}>{children}</div>
      </Slide>
    );
  },
);

export default OmniPanelView;
