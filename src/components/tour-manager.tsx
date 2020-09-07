import React from 'react';
import { Typography, createMuiTheme } from '@material-ui/core';
import { SpotlightValue } from './spotlight-value';
import { ReactourStep } from 'reactour';

type stepStyle = {
  backgroundColor: string;
  color: string;
  borderRadius: string;
};

type stepObject = {
  selector: string;
  content: string;
  action: () => void;
  style?: stepStyle;
};

type tourContent = {
  [primaryKey: string]: stepObject;
};

export const createTourSteps = (args: {
  setTourSettingsAndOmniPanel: (
    isSettingsVisible: boolean,
    isOmniPanelVisible: boolean,
    clearSpotlight?: boolean | undefined,
  ) => void;
  setTourShowOmniPanel: (view: typeof OmniPanelViewIndex) => void;
  OmniPanelViewIndex: any;
  doorSpotlight: SpotlightValue<string> | undefined;
  setDoorSpotlight: React.Dispatch<React.SetStateAction<SpotlightValue<string> | undefined>>;
}) => {
  const {
    setTourSettingsAndOmniPanel,
    setTourShowOmniPanel,
    OmniPanelViewIndex,
    doorSpotlight,
    setDoorSpotlight,
  } = args;

  const theme = createMuiTheme();
  const stepStyle: stepStyle = {
    backgroundColor: theme.palette.primary.light,
    color: theme.palette.info.contrastText,
    borderRadius: '5px',
  };

  const createContent = (str: string): React.ReactNode => {
    return <Typography variant="h6">{str}</Typography>;
  };

  const tourContent: tourContent = {
    dashboard: {
      selector: '',
      content: 'Welcome to RoMi Dashboard',
      action: () => setTourSettingsAndOmniPanel(false, false, true),
    },
    zoomButtons: {
      selector: '[class="leaflet-control-zoom leaflet-bar leaflet-control"]',
      content:
        'Click on the zoom buttons to change the view of the floor plan. Alternatively, the scroll button on your mouse would work too!',
      action: () => setTourSettingsAndOmniPanel(false, false, true),
    },
    floorPlan: {
      selector: '[class= "leaflet-control-layers leaflet-control"]',
      content:
        'Use the floor plan button to switch between available levels and enabling / disabling the view of different components.',
      action: () => setTourSettingsAndOmniPanel(false, false, true),
    },
    leaflet: {
      selector: '[class="leaflet-image-layer leaflet-zoom-animated"]',
      content:
        'Clicking individual components like doors, robots, lifts on the map will open up its corresponding information tab in the omnipanel.',
      action: () => setTourSettingsAndOmniPanel(false, false, true),
    },
    omnipanelButton: {
      selector: '[data-name="omnipanel-button"]',
      content:
        'The Omnipanel Button shows the different panel options available in the dashboard. Clicking each item would list different information about it!',
      action: () => setTourShowOmniPanel(OmniPanelViewIndex.MainMenu),
    },
    mainMenu: {
      selector: '[data-component="MainMenu"]',
      content: 'Each Panel contains a list of the available items and their corresponding states.',
      action: () => setTourShowOmniPanel(OmniPanelViewIndex.MainMenu),
    },
    doorsPanel: {
      selector: '',
      content: 'Let us take a look into the🚪Doors Panel',
      action: () => setTourShowOmniPanel(OmniPanelViewIndex.Doors),
    },
    doorTab: {
      selector: '[data-name="main_door"]',
      content: 'Here is an example of what you will see when a door tab is expanded!',
      action: () => {
        setTourShowOmniPanel(OmniPanelViewIndex.Doors);
        if (!doorSpotlight) {
          setDoorSpotlight({ value: 'main_door' });
        }
      },
    },
    commandsPanel: {
      selector: '',
      content:
        'The Commands Panel allows you to send different types of requests that will be handled by RoMi.',
      action: () => setTourShowOmniPanel(OmniPanelViewIndex.Commands),
    },
    loopRequest: {
      selector: '[data-component="LoopForm"]',
      content:
        'An example is the Loop Request which can be iterated multiple times. RoMi will assign the most suitable robot to perform the task at the point of request.',
      action: () => setTourShowOmniPanel(OmniPanelViewIndex.Commands),
    },
    settingsButton: {
      selector: '',
      content: 'The Settings Button opens up the drawer for different dashboard settings',
      action: () => setTourSettingsAndOmniPanel(true, false),
    },
    trajAnim: {
      selector: '.MuiDrawer-paper',
      content:
        'Finally, Trajectory Animations can be changed using the options available. Look out for new features ahead!',
      action: () => setTourSettingsAndOmniPanel(true, false),
    },
  };

  const tourSteps: ReactourStep[] = [];

  for (var key in tourContent) {
    let content = createContent(tourContent[key].content);
    tourSteps.push({
      selector: tourContent[key].selector,
      content: content,
      action: tourContent[key].action,
      style: stepStyle,
    });
  }

  return tourSteps;
};
