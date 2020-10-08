import { makeStyles } from '@material-ui/core';
import Slide from '@material-ui/core/Slide';
import React, { ReactElement } from 'react';
import { OmniPanelViewProps } from './omni-panel-view';

const useStyles = makeStyles(() => ({
  container: {
    position: 'relative',
    overflow: 'hidden',
  },
  viewContainer: {
    width: '100%',
    height: '100%',
    position: 'absolute',
  },
}));

export interface OmniPanelProps extends React.HTMLAttributes<HTMLDivElement> {
  view: number | string;
  children: React.ReactElement<OmniPanelViewProps>[] | React.ReactElement<OmniPanelViewProps>;
  timeout?: number;
  mountOnEnter?: boolean;
  unmountOnExit?: boolean;
}

export const OmniPanel = React.forwardRef(function (
  props: OmniPanelProps,
  ref: React.Ref<HTMLDivElement>,
) {
  const { view, className, timeout, mountOnEnter, unmountOnExit, children, ...otherProps } = props;
  const classes = useStyles();

  const renderView = (child: ReactElement<OmniPanelViewProps>) => (
    <Slide
      key={child.props.id}
      direction="left"
      in={view === child.props.id}
      appear={false}
      timeout={timeout}
      mountOnEnter={mountOnEnter}
      unmountOnExit={unmountOnExit}
    >
      <div className={classes.viewContainer}>{child}</div>
    </Slide>
  );
  return (
    <div ref={ref} className={`${classes.container} ${className}`} {...otherProps}>
      {Array.isArray(children) ? children.map(renderView) : renderView(children)}
    </div>
  );
});

export default OmniPanel;
