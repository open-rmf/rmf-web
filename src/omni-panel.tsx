import { Button, ButtonGroup, makeStyles } from '@material-ui/core';
import {
  Close as CloseIcon,
  KeyboardBackspace as BackIcon,
} from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import DoorsPanel from './doors-panel';
import LiftsPanel from './lifts-panel';
import MainMenu, { MainMenuProps } from './main-menu';
import RobotsPanel from './robots-panel';
import * as CSSUtils from './util/css-utils';

const useStyles = makeStyles(() => ({
  container: {
    display: 'flex',
    flexFlow: 'column',
  },
}));

export enum OmniPanelView {
  MainMenu,
  Doors,
  Lifts,
  Robots,
}

interface OmniPanelProps {
  className?: string;
  classes?: {
    navigation?: string;
    backButton?: string;
    closeButton?: string;
  };
  initialView: OmniPanelView;
  buildingMap: RomiCore.BuildingMap;
  doorStates: { [key: string]: RomiCore.DoorState };
  liftStates: { [key: string]: RomiCore.LiftState };
  fleets: RomiCore.FleetState[];
  onClose?: () => void;
}

class ViewMapNode {
  constructor(public value: OmniPanelView, public parent?: ViewMapNode) {}

  addChild(view: OmniPanelView): ViewMapNode {
    return new ViewMapNode(view, this);
  }
}

type ViewMap = { [key: number]: ViewMapNode };

function makeViewMap(): ViewMap {
  const viewMap: ViewMap = {};
  const root = new ViewMapNode(OmniPanelView.MainMenu);
  viewMap[OmniPanelView.MainMenu] = root;
  viewMap[OmniPanelView.Doors] = root.addChild(OmniPanelView.Doors);
  viewMap[OmniPanelView.Lifts] = root.addChild(OmniPanelView.Lifts);
  viewMap[OmniPanelView.Robots] = root.addChild(OmniPanelView.Robots);
  return viewMap;
}

const viewMap = makeViewMap();

export default function OmniPanel(props: OmniPanelProps): JSX.Element {
  const classes = useStyles();
  const [currentView, setCurrentView] = React.useState(
    viewMap[props.initialView],
  );

  function handleBackClick() {
    if (!currentView.parent) {
      return props.onClose && props.onClose();
    }
    setCurrentView(currentView.parent);
  }

  function handleCloseClick() {
    props.onClose && props.onClose();
  }

  const mainMenuProps: MainMenuProps = {
    onDoorsClick: () => {
      setCurrentView(viewMap[OmniPanelView.Doors]);
    },
    onLiftsClick: () => {
      setCurrentView(viewMap[OmniPanelView.Lifts]);
    },
    onRobotsClick: () => {
      setCurrentView(viewMap[OmniPanelView.Robots]);
    },
  };

  return (
    <div className={`${classes.container} ${props.className}`}>
      <ButtonGroup
        className={CSSUtils.className(props.classes, 'navigation')}
        variant="text"
        fullWidth
      >
        <Button
          className={CSSUtils.className(props.classes, 'backButton')}
          size="large"
          onClick={handleBackClick}
        >
          <BackIcon />
        </Button>
        <Button
          className={CSSUtils.className(props.classes, 'closeButton')}
          size="large"
          onClick={handleCloseClick}
        >
          <CloseIcon />
        </Button>
      </ButtonGroup>

      {currentView.value === OmniPanelView.MainMenu && (
        <MainMenu
          onDoorsClick={mainMenuProps.onDoorsClick}
          onLiftsClick={mainMenuProps.onLiftsClick}
          onRobotsClick={mainMenuProps.onRobotsClick}
        />
      )}

      {currentView.value === OmniPanelView.Doors && (
        <DoorsPanel
          buildingMap={props.buildingMap}
          doorStates={props.doorStates}
        />
      )}

      {currentView.value === OmniPanelView.Lifts && (
        <LiftsPanel
          buildingMap={props.buildingMap}
          liftStates={props.liftStates}
        />
      )}

      {currentView.value === OmniPanelView.Robots && (
        <RobotsPanel fleets={props.fleets} />
      )}
    </div>
  );
}
