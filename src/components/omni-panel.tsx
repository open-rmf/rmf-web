import { Button, ButtonGroup, makeStyles, Slide } from '@material-ui/core';
import { Close as CloseIcon, KeyboardBackspace as BackIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { OmniPanelViewProps } from './omni-panel-view';

const useStyles = makeStyles(() => ({
  container: {
    display: 'flex',
    flexFlow: 'column',
  },

  viewContainer: {
    height: '100%',
    overflowX: 'hidden',
    position: 'relative',
  },

  viewContainer2: {
    width: '100%',
    height: '100%',
    position: 'absolute',
  },
}));

export interface OmniPanelProps extends React.HTMLAttributes<HTMLDivElement> {
  view: number;
  transport?: Readonly<RomiCore.Transport>;
  classes?: {
    navigation?: string;
    backButton?: string;
    closeButton?: string;
  };
  onBack?: (current: number) => void;
  onClose?: () => void;
  children?: React.ReactElement<OmniPanelViewProps>[];
}

export const OmniPanel = React.forwardRef(function(
  props: OmniPanelProps,
  ref: React.Ref<HTMLDivElement>,
): React.ReactElement {
  const { view, classes: innerClasses, onBack, onClose, children, ...otherProps } = props;
  const classes = useStyles();

  const className = (() =>
    `${otherProps.className || ''} ${classes.container} ${props.className}`)();

  function handleBackClick() {
    props.onBack && props.onBack(props.view);
  }

  function handleCloseClick() {
    props.onClose && props.onClose();
  }

  return (
    <div {...otherProps} ref={ref} className={className}>
      <ButtonGroup className={props.classes?.navigation} variant="text" fullWidth>
        <Button className={props.classes?.backButton} size="large" onClick={handleBackClick}>
          <BackIcon />
        </Button>
        <Button className={props.classes?.closeButton} size="large" onClick={handleCloseClick}>
          <CloseIcon />
        </Button>
      </ButtonGroup>
      <div className={classes.viewContainer}>
        {props.children?.map(child => (
          <Slide
            key={child.props.id}
            direction="left"
            in={child.props.id === view}
            mountOnEnter
            unmountOnExit
            appear={false}
          >
            <div className={classes.viewContainer2}>{child}</div>
          </Slide>
        ))}
      </div>
    </div>
  );
});

export default OmniPanel;
