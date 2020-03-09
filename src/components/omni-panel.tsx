import { Button, ButtonGroup, makeStyles } from '@material-ui/core';
import { Close as CloseIcon, KeyboardBackspace as BackIcon } from '@material-ui/icons';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import debug from 'debug';
import React from 'react';
import DoorStateManager from '../door-state-manager';
import FleetManager from '../fleet-manager';
import LiftStateManager from '../lift-state-manager';
import * as CSSUtils from '../util/css-utils';
import DoorsPanel from './doors-panel';
import LiftsPanel from './lifts-panel';
import MainMenu, { MainMenuProps } from './main-menu';
import RobotsPanel from './robots-panel';

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
  transport?: Readonly<RomiCore.Transport>;
  className?: string;
  classes?: {
    navigation?: string;
    backButton?: string;
    closeButton?: string;
  };
  initialView: Readonly<OmniPanelView>;
  buildingMap?: Readonly<RomiCore.BuildingMap>;
  doorStateManager: Readonly<DoorStateManager>;
  liftStateManager: Readonly<LiftStateManager>;
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
  const [doorStates, setDoorStates] = React.useState<Readonly<Record<string, RomiCore.DoorState>>>(
    {},
  );
  const [liftStates, setLiftStates] = React.useState<Readonly<Record<string, RomiCore.LiftState>>>(
    {},
  );
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

  // start subscription to state publications
  React.useEffect(() => {
    props.transport && props.doorStateManager.startSubscription(props.transport);
  }, [props.transport, props.doorStateManager]);
  React.useEffect(() => {
    props.transport && props.liftStateManager.startSubscription(props.transport);
  }, [props.transport, props.liftStateManager]);
  React.useEffect(() => {
    props.transport && props.fleetManager.startSubscription(props.transport);
  }, [props.transport, props.fleetManager]);

  // update state only when relevant view is active
  React.useEffect(() => {
    if (currentView.value !== OmniPanelView.Doors) {
      return;
    }
    setDoorStates(props.doorStateManager.doorStates());
    const listener = () => setDoorStates(props.doorStateManager.doorStates());
    props.doorStateManager.on('updated', listener);
    debug.log('started tracking door states');
    return () => {
      props.doorStateManager.off('updated', listener);
      debug.log('stopped tracking door states');
    };
  }, [currentView.value, props.doorStateManager]);

  React.useEffect(() => {
    if (currentView.value !== OmniPanelView.Lifts) {
      return;
    }
    setLiftStates(props.liftStateManager.liftStates());
    const listener = () => setLiftStates(props.liftStateManager.liftStates());
    props.liftStateManager.on('updated', listener);
    debug.log('started tracking lift states');
    return () => {
      props.liftStateManager.off('updated', listener);
      debug.log('stopped tracking lift states');
    };
  }, [currentView.value, props.liftStateManager]);

  React.useEffect(() => {
    if (currentView.value !== OmniPanelView.Robots) {
      return;
    }
    setFleets(props.fleetManager.fleets());
    const listener = () => setFleets(props.fleetManager.fleets());
    props.fleetManager.on('updated', listener);
    debug.log('started tracking fleet states');
    return () => {
      props.fleetManager.off('updated', listener);
      debug.log('stopped tracking fleet states');
    };
  }, [currentView.value, props.fleetManager]);

  const doors = props.buildingMap ? props.buildingMap.levels.flatMap(level => level.doors) : [];
  const lifts = props.buildingMap ? props.buildingMap.lifts : [];

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
