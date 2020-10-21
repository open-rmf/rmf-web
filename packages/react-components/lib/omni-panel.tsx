import { makeStyles } from '@material-ui/core';
import Button from '@material-ui/core/Button';
import ButtonGroup from '@material-ui/core/ButtonGroup';
import Slide from '@material-ui/core/Slide';
import CloseIcon from '@material-ui/icons/Close';
import HomeIcon from '@material-ui/icons/Home';
import BackIcon from '@material-ui/icons/KeyboardBackspace';
import React, { ReactElement } from 'react';
import { OmniPanelViewProps } from './omni-panel-view';

const useStyles = makeStyles(() => ({
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
    position: 'absolute',
  },
  navigationButton: {
    borderRadius: 'inherit',
  },
}));

export interface OmniPanelProps extends React.HTMLAttributes<HTMLDivElement> {
  view: number | string;
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
    view,
    children,
    variant,
    timeout,
    mountOnEnter,
    unmountOnExit,
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
      in={view === child.props.viewId}
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
        <ButtonGroup className={classes_.navigationButton} variant="text" fullWidth>
          <Button
            className={classes_.navigationButton}
            size="large"
            onClick={onBack}
            data-testid="back-button"
          >
            <BackIcon />
          </Button>
          <Button
            className={classes_.navigationButton}
            size="large"
            onClick={onHome}
            data-testid="home-button"
          >
            <HomeIcon />
          </Button>
          {variant === 'backHomeClose' && (
            <Button
              className={classes_.navigationButton}
              size="large"
              onClick={onClose}
              data-testid="close-button"
            >
              <CloseIcon />
            </Button>
          )}
        </ButtonGroup>
        <div className={classes_.viewContainer}>
          {Array.isArray(children) ? children.map(renderView) : renderView(children)}
        </div>
      </div>
    </div>
  );
};

export default OmniPanel;
