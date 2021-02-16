import { Divider, IconButton, makeStyles } from '@material-ui/core';
import Slide from '@material-ui/core/Slide';
import CloseIcon from '@material-ui/icons/Close';
import HomeIcon from '@material-ui/icons/Home';
import BackIcon from '@material-ui/icons/KeyboardBackspace';
import React, { ReactElement } from 'react';
import { OmniPanelViewProps } from './omni-panel-view';

const useStyles = makeStyles((theme) => ({
  mainContainer: {
    width: '100%',
    height: '100%',
    display: 'flex',
    flexFlow: 'column',
    borderRadius: 'inherit',
  },
  viewContainer: {
    width: '100%',
    height: '100%',
    position: 'relative',
    overflow: 'hidden',
  },
  viewContainer2: {
    width: '100%',
    height: '100%',
    position: 'absolute',
    overflow: 'auto',
  },
  navigationButton: {
    borderRadius: 'inherit',
    flexGrow: 1,
    color: theme.palette.text.primary,
  },
  navigationButtonContainer: {
    display: 'flex',
  },
}));

export interface OmniPanelProps extends React.HTMLProps<HTMLDivElement> {
  stack: (number | string)[];
  children: React.ReactElement<OmniPanelViewProps>[] | React.ReactElement<OmniPanelViewProps>;
  variant?: 'backHome' | 'backHomeClose';
  timeout?: number;
  mountOnEnter?: boolean;
  unmountOnExit?: boolean;
  onBack?: React.MouseEventHandler<HTMLButtonElement>;
  onHome?: React.MouseEventHandler<HTMLButtonElement>;
  onClose?: React.MouseEventHandler<HTMLButtonElement>;
}

export const OmniPanel = (props: OmniPanelProps): JSX.Element => {
  const {
    stack,
    children,
    variant,
    timeout,
    mountOnEnter,
    unmountOnExit = true,
    onBack,
    onHome,
    onClose,
    ...otherProps
  } = props;
  const classes_ = useStyles();

  const renderView = (child: ReactElement<OmniPanelViewProps>) => (
    <Slide
      key={child.props.viewId}
      direction="left"
      in={stack[stack.length - 1] === child.props.viewId}
      appear={false}
      timeout={timeout}
      mountOnEnter={mountOnEnter}
      unmountOnExit={unmountOnExit}
    >
      <div className={classes_.viewContainer2}>{child}</div>
    </Slide>
  );
  return (
    <div {...otherProps}>
      <div className={classes_.mainContainer}>
        <div className={classes_.navigationButtonContainer}>
          <IconButton className={classes_.navigationButton} onClick={onBack} aria-label="Back">
            <BackIcon />
          </IconButton>
          <Divider orientation="vertical" />
          <IconButton className={classes_.navigationButton} onClick={onHome} aria-label="Home">
            <HomeIcon />
          </IconButton>
          {variant === 'backHomeClose' && (
            <>
              <Divider orientation="vertical" />
              <IconButton
                className={classes_.navigationButton}
                onClick={onClose}
                aria-label="Close"
              >
                <CloseIcon />
              </IconButton>
            </>
          )}
        </div>
        <Divider />
        <div className={classes_.viewContainer}>
          {Array.isArray(children) ? children.map(renderView) : renderView(children)}
        </div>
      </div>
    </div>
  );
};

export default OmniPanel;
