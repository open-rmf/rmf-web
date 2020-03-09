import { Button, ButtonGroup, makeStyles } from '@material-ui/core';
import { Close as CloseIcon, KeyboardBackspace as BackIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import DoorsPanel from './doors-panel';
import LiftsPanel from './lifts-panel';
import MainMenu, { MainMenuProps } from './main-menu';
import RobotsPanel from './robots-panel';
import * as CSSUtils from '../util/css-utils';
import FleetManager from '../fleet-manager';

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
  transport?: RomiCore.Transport;
  className?: string;
  classes?: {
    navigation?: string;
    backButton?: string;
    closeButton?: string;
  };
  initialView: OmniPanelView;
  buildingMap?: RomiCore.BuildingMap;
  doorStates?: { [key: string]: RomiCore.DoorState };
  liftStates?: { [key: string]: RomiCore.LiftState };
  fleetManager: Readonly<FleetManager>;
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
  const [currentView, setCurrentView] = React.useState(viewMap[props.initialView]);
  const [fleets, setFleets] = React.useState<readonly RomiCore.FleetState[]>([]);

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

  React.useEffect(() => {
    if (currentView.value !== OmniPanelView.Robots) {
      return;
    }
    setFleets(props.fleetManager.fleets());
    const listener = () => setFleets(props.fleetManager.fleets());
    props.fleetManager.on('data', listener);
    return () => { props.fleetManager.off('data', listener); };
  }, [currentView.value, props.fleetManager]);

  const doors = props.buildingMap ? props.buildingMap.levels.flatMap(level => level.doors) : [];
  const doorStates = props.doorStates ? props.doorStates : {};
  const lifts = props.buildingMap ? props.buildingMap.lifts : [];
  const liftStates = props.liftStates ? props.liftStates : {};

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

      {currentView.value === OmniPanelView.Doors && props.buildingMap && (
        <DoorsPanel transport={props.transport} doors={doors} doorStates={doorStates} />
      )}

      {currentView.value === OmniPanelView.Lifts && props.buildingMap && (
        <LiftsPanel lifts={lifts} liftStates={liftStates} />
      )}

      {currentView.value === OmniPanelView.Robots && <RobotsPanel fleets={fleets} />}
    </div>
  );
}
